// SPDX-License-Identifier: GPL-3.0-only
#include "io/midi_port.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace blofeld::vi {
namespace {
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
constexpr auto index_mask = MidiPort::capacity - 1;
static_assert((MidiPort::capacity & index_mask) == 0);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
}

std::optional<MidiPort::Event> MidiPort::Queue::peek() const noexcept {
  const auto at = read.load(std::memory_order_relaxed);
  if (at == written.load(std::memory_order_acquire)) { return {}; }
  return events[at & index_mask];
}

void MidiPort::Queue::pop() noexcept {
  read.store(read.load(std::memory_order_relaxed) + 1, std::memory_order_release);
}

bool MidiPort::fail(const char *message) noexcept {
  const char *empty = nullptr;
  error_.compare_exchange_strong(empty, message, std::memory_order_release);
  return false;
}

bool MidiPort::write_input(const std::span<const std::uint8_t> bytes,
                           const std::uint64_t ticks) noexcept {
  if (error() != nullptr) { return false; }
  if (bytes.empty()) { return true; }
  if (ticks < last_input_) { return fail("MIDI input timestamps moved backwards"); }
  const auto at = input_.written.load(std::memory_order_relaxed);
  const auto used = at - input_.read.load(std::memory_order_acquire);
  if (bytes.size() > capacity - used) { return fail("MIDI input queue is full"); }
  const auto first = std::max(ticks, next_input_);
  if (bytes.size() > maximum - at || bytes.size() > (maximum - first) / byte_ticks) {
    return fail("MIDI input timeline or queue sequence is not representable");
  }
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    input_.events[(at + index) & index_mask] = {.ticks = first + (index * byte_ticks),
                                                .byte = bytes[index]};
  }
  last_input_ = ticks;
  next_input_ = first + bytes.size() * byte_ticks;
  input_.written.store(at + bytes.size(), std::memory_order_release);
  return true;
}

std::optional<MidiPort::Event> MidiPort::peek_input() const noexcept {
  return input_.peek();
}

void MidiPort::pop_input() noexcept {
  input_.pop();
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
  output_.written.store(at + 1, std::memory_order_release);
  return true;
}

std::optional<MidiPort::Event> MidiPort::read_output() noexcept {
  const auto event = output_.peek();
  if (event) { output_.pop(); }
  return event;
}
}
