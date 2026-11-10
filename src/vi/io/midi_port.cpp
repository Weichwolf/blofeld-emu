// SPDX-License-Identifier: GPL-3.0-only
#include "io/midi_port.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>

namespace blofeld::vi {
namespace {
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr auto index_mask = MidiPort::capacity - 1;
static_assert((MidiPort::capacity & index_mask) == 0);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<unsigned>::is_always_lock_free);

struct Waiting {
  std::atomic<unsigned> &waiters;

  explicit Waiting(std::atomic<unsigned> &value) : waiters(value) {
    waiters.fetch_add(1, std::memory_order_seq_cst);
  }

  ~Waiting() { waiters.fetch_sub(1, std::memory_order_seq_cst); }
};
}

std::optional<MidiPort::Event> MidiPort::Input::peek(const Receiver receiver) const noexcept {
  const auto at = (receiver == Receiver::mcu ? mcu : dsp).load(std::memory_order_relaxed);
  if (at == written.load(std::memory_order_acquire)) { return {}; }
  return events[at & index_mask];
}

void MidiPort::Input::pop(const Receiver receiver) noexcept {
  auto &cursor = receiver == Receiver::mcu ? mcu : dsp;
  cursor.store(cursor.load(std::memory_order_relaxed) + 1, std::memory_order_seq_cst);
}

std::optional<MidiPort::Event> MidiPort::Queue::peek() const noexcept {
  const auto at = read.load(std::memory_order_relaxed);
  if (at == written.load(std::memory_order_acquire)) { return {}; }
  return events[at & index_mask];
}

void MidiPort::Queue::pop() noexcept {
  read.store(read.load(std::memory_order_relaxed) + 1, std::memory_order_release);
}

void MidiPort::signal() noexcept {
  changed_.fetch_add(1, std::memory_order_seq_cst);
  changed_.notify_all();
}

void MidiPort::notify() noexcept {
  if (input_waiters_.load(std::memory_order_seq_cst) != 0 ||
      space_waiters_.load(std::memory_order_seq_cst) != 0 ||
      output_waiters_.load(std::memory_order_seq_cst) != 0) {
    signal();
  }
}

template <typename Ready>
bool MidiPort::await(std::atomic<unsigned> &waiters,
                     const std::stop_token &stop,
                     Ready ready) noexcept {
  if (stop.stop_requested() || error() != nullptr) { return false; }
  if (ready()) { return true; }
  // Each blocked owner retains its registration until its own exit.
  const Waiting registered{waiters};
  const std::stop_callback cancelled(stop, [this] { signal(); });
  for (;;) {
    const auto observed = changed_.load(std::memory_order_seq_cst);
    if (stop.stop_requested() || error() != nullptr) { return false; }
    if (ready()) { return true; }
    changed_.wait(observed, std::memory_order_acquire);
  }
}

bool MidiPort::fail(const char *message) noexcept {
  const char *empty = nullptr;
  error_.compare_exchange_strong(empty, message, std::memory_order_release);
  signal();
  return false;
}

bool MidiPort::write_input(const std::span<const std::uint8_t> bytes,
                           const std::uint64_t ticks) noexcept {
  return admit(bytes, ticks, nullptr);
}

bool MidiPort::write_input(const std::span<const std::uint8_t> bytes,
                           const std::uint64_t ticks,
                           const std::stop_token &stop) noexcept {
  return admit(bytes, ticks, &stop);
}

bool MidiPort::admit(const std::span<const std::uint8_t> bytes,
                     const std::uint64_t ticks,
                     const std::stop_token *stop) noexcept {
  if (stop != nullptr && stop->stop_requested()) { return false; }
  if (error() != nullptr) { return false; }
  if (phase_.load(std::memory_order_seq_cst) == Phase::ended) {
    return fail("MIDI input is already finished");
  }
  if (bytes.empty()) { return true; }
  if (ticks < last_input_) { return fail("MIDI input timestamps moved backwards"); }
  const auto at = input_.written.load(std::memory_order_relaxed);
  if (bytes.size() > capacity) { return fail("MIDI input packet exceeds queue capacity"); }
  const auto first = std::max(ticks, next_input_);
  if (input_mode_ == InputMode::stream && first < input_until_.load(std::memory_order_seq_cst)) {
    return fail("MIDI input precedes its committed stream frontier");
  }
  if (first < time_.load(std::memory_order_acquire)) {
    return fail("MIDI character starts before published instrument time");
  }
  if (bytes.size() > maximum - at || bytes.size() > (maximum - first) / byte_ticks) {
    return fail("MIDI input timeline or queue sequence is not representable");
  }
  const auto room = [&] {
    const auto retired = std::min(input_.mcu.load(std::memory_order_seq_cst),
                                  input_.dsp.load(std::memory_order_seq_cst));
    return bytes.size() <= capacity - (at - retired);
  };
  if (stop != nullptr) {
    if (!await(space_waiters_, *stop, room)) { return false; }
  } else if (!room()) {
    return fail("MIDI input queue is full");
  }
  if (first < time_.load(std::memory_order_acquire)) {
    return fail("MIDI character starts before published instrument time");
  }
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    input_.events[(at + index) & index_mask] = {.ticks = first + (index * byte_ticks),
                                                .byte = bytes[index]};
  }
  last_input_ = ticks;
  next_input_ = first + bytes.size() * byte_ticks;
  input_.written.store(at + bytes.size(), std::memory_order_release);
  if (input_mode_ == InputMode::stream) {
    input_until_.store(next_input_, std::memory_order_seq_cst);
    notify();
  }
  return true;
}

std::optional<std::uint64_t> MidiPort::input_limit() const noexcept {
  if (input_mode_ == InputMode::live || phase_.load(std::memory_order_seq_cst) == Phase::ended) {
    return {};
  }
  return input_until_.load(std::memory_order_seq_cst);
}

bool MidiPort::advance_input(const std::uint64_t ticks) noexcept {
  if (error() != nullptr) { return false; }
  if (input_mode_ != InputMode::stream || phase_.load(std::memory_order_seq_cst) == Phase::ended) {
    return fail("MIDI input has no active stream frontier");
  }
  if (ticks < input_until_.load(std::memory_order_seq_cst)) {
    return fail("MIDI stream frontier moved backwards");
  }
  input_until_.store(ticks, std::memory_order_seq_cst);
  notify();
  return true;
}

bool MidiPort::finish_input() noexcept {
  if (error() != nullptr) { return false; }
  if (input_mode_ != InputMode::stream) {
    return fail("Live MIDI input cannot declare stream EOF");
  }
  phase_.store(Phase::ended, std::memory_order_seq_cst);
  notify();
  return true;
}

void MidiPort::abort_input() noexcept {
  static_cast<void>(fail("MIDI input source aborted"));
}

bool MidiPort::await_input(const std::uint64_t ticks, const std::stop_token &stop) noexcept {
  if (input_mode_ == InputMode::live) { return !stop.stop_requested() && error() == nullptr; }
  return await(input_waiters_, stop, [&] {
    return phase_.load(std::memory_order_seq_cst) == Phase::ended ||
           input_until_.load(std::memory_order_seq_cst) > ticks;
  });
}

std::optional<MidiPort::Event> MidiPort::peek_input(const Receiver receiver) const noexcept {
  return input_.peek(receiver);
}

void MidiPort::pop_input(const Receiver receiver) noexcept {
  input_.pop(receiver);
  notify();
}

bool MidiPort::write_output(const Event event) noexcept {
  if (error() != nullptr) { return false; }
  if (event.ticks < last_output_) { return fail("MIDI output timestamps moved backwards"); }
  const auto at = output_.written.load(std::memory_order_relaxed);
  if (at - output_.read.load(std::memory_order_acquire) == capacity) {
    return fail("MIDI output queue is full");
  }
  if (at == maximum) { return fail("MIDI output queue sequence is not representable"); }
  output_.events[at & index_mask] = event;
  last_output_ = event.ticks;
  output_.written.store(at + 1, std::memory_order_seq_cst);
  notify();
  return true;
}

std::optional<MidiPort::Event> MidiPort::read_output() noexcept {
  const auto event = output_.peek();
  if (event) { output_.pop(); }
  return event;
}

bool MidiPort::await_output(const std::stop_token &stop) noexcept {
  return await(output_waiters_, stop, [this] {
    return output_.written.load(std::memory_order_seq_cst) !=
           output_.read.load(std::memory_order_relaxed);
  });
}
}
