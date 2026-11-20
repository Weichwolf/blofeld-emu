// SPDX-License-Identifier: GPL-3.0-only
#include "io/coupling.hpp"

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/effects.hpp"
#include "mcu/epoch.hpp"
#include "mcu/input_history.hpp"
#include "mcu/output.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>

namespace blofeld::vi {
namespace {
constexpr unsigned known_shift = 8, direction_shift = 16;

constexpr std::size_t index(const mcu::Epoch::Signal signal) noexcept {
  return static_cast<std::size_t>(signal);
}
}

Coupling::Coupling(PeerPort &peer,
                   Shi &shi,
                   DspReset &reset,
                   ExecutionTime &time,
                   MidiPort &midi,
                   std::stop_token stop,
                   const std::uint64_t window)
    : peer_(peer),
      shi_(shi),
      reset_(reset),
      time_(time),
      midi_(midi),
      stop_(std::move(stop)),
      window_(window),
      cancellation_(stop_, Cancel{this}) {
  midi_.wake(this, [](void *owner) noexcept { static_cast<Coupling *>(owner)->notify(); });
}

Coupling::~Coupling() {
  midi_.wake(nullptr, nullptr);
}

void Coupling::Cancel::operator()() const noexcept {
  owner->notify();
}

void Coupling::notify() noexcept {
  auto changed = std::atomic_ref(changed_);
  changed.fetch_add(1, std::memory_order_seq_cst);
  if (waiting_.load(std::memory_order_seq_cst) != 0) { changed.notify_all(); }
}

void Coupling::wait(const std::uint64_t observed) noexcept {
  waiting_.fetch_add(1, std::memory_order_seq_cst);
  if (!stopped()) {
#if defined(BLOFELD_CENSUS)
    const Census::Wait observed_wait;
#endif
    std::atomic_ref(changed_).wait(observed, std::memory_order_acquire);
  }
  waiting_.fetch_sub(1, std::memory_order_seq_cst);
}

void Coupling::wait_checkpoint(const std::uint64_t observed) noexcept {
  if (!midi_.before_checkpoint_wait(
          {.generation = changed_, .observed = observed, .command = checkpoint_sequence_})) {
    wait(observed);
  }
}

bool Coupling::fail(const char *const message) noexcept {
  const char *empty = nullptr;
  error_.compare_exchange_strong(empty, message, std::memory_order_release);
  notify();
  return false;
}

void Coupling::bind() noexcept {
  bound_.store(true, std::memory_order_release);
  notify();
}

void Coupling::publish(const std::span<const mcu::Effects::Entry> entries,
                       const std::uint64_t frontier) noexcept {
  published_.store(entries.size(), std::memory_order_release);
  frontier_.store(frontier, std::memory_order_release);
  notify();
}

void Coupling::request(const std::uint64_t ticks) noexcept {
  const auto old = frontier_.load(std::memory_order_relaxed);
  if (ticks > old) { frontier_.store(ticks, std::memory_order_release); }
  notify();
}

std::optional<Coupling::Input> Coupling::input(const std::uint64_t seen) const noexcept {
  const auto serial = message_.load(std::memory_order_acquire);
  if (serial == 0 || serial == seen) { return {}; }
  return input_;
}

void Coupling::acknowledge(const std::uint64_t serial) noexcept {
  auto expected = serial;
  if (!message_.compare_exchange_strong(expected, 0, std::memory_order_acq_rel)) {
    static_cast<void>(fail("DSP input acknowledgement does not match the pending event"));
  }
  notify();
}

std::size_t Coupling::consumed() const noexcept {
  return consumed_.load(std::memory_order_acquire);
}

std::uint64_t Coupling::cut() const noexcept {
  return cut_.load(std::memory_order_acquire);
}

std::uint64_t Coupling::known() const noexcept {
  return known_.load(std::memory_order_acquire);
}

bool Coupling::pause_after(const std::uint64_t ticks) const noexcept {
  const auto command = checkpoint_request_.load(std::memory_order_acquire);
  return command != 0 && checkpoint_ready_.load(std::memory_order_acquire) == command &&
         (known() > ticks || (reset_.state().held && known() == ticks)) &&
         message_.load(std::memory_order_acquire) == 0;
}

void Coupling::checkpoint(const std::uint64_t ticks) noexcept {
  if (checkpoint_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    static_cast<void>(fail("MCU checkpoint sequence is exhausted"));
    return;
  }
  checkpoint_ticks_.store(ticks, std::memory_order_relaxed);
  checkpoint_request_.store(++checkpoint_sequence_, std::memory_order_release);
  notify();
}

void Coupling::cancel_checkpoint() noexcept {
  checkpoint_request_.store(0, std::memory_order_release);
  notify();
}

void Coupling::replace(const std::uint64_t origin) noexcept {
  published_.store(0, std::memory_order_release);
  consumed_.store(0, std::memory_order_release);
  frontier_.store(origin, std::memory_order_release);
  generation_.fetch_add(1, std::memory_order_release);
  cancel_checkpoint();
}

void Coupling::refresh() noexcept {
  const auto generation = generation_.load(std::memory_order_acquire);
  if (generation == receiver_generation_) { return; }
  receiver_generation_ = generation;
  cursor_ = 0;
  consumed_.store(0, std::memory_order_release);
}

void Coupling::attained(const std::uint64_t exclusive) noexcept {
  const auto old = known_.load(std::memory_order_relaxed);
  if (exclusive > old) {
    known_.store(exclusive, std::memory_order_release);
    notify();
  }
}

mcu::InputHistory::Values Coupling::sample() {
  using Signal = mcu::Epoch::Signal;
  mcu::InputHistory::Values values{};
  const auto view = peer_.view();
  if (const auto driven = std::uint64_t{view.data.mask}; driven != 0) {
    values[index(Signal::data)] = (driven << direction_shift) |
                                  ((view.data.known & driven) << known_shift) |
                                  (view.data.value & driven);
  }
  values[index(Signal::interrupt)] = std::uint64_t{view.ready};
  values[index(Signal::generation)] = view.generation;
  values[index(Signal::announce)] = view.announced ? view.generation : 0;
  values[index(Signal::reply)] = std::uint64_t{view.reply};
  if (const auto mode = shi_.configuration(true)) {
    values[index(Signal::mode)] = mode->bits | (std::uint64_t{mode->falling} << known_shift) |
                                  (std::uint64_t{mode->continuous} << (known_shift + 1));
  }
  return values;
}

bool Coupling::outputs(const std::uint64_t ticks) {
  if (stopped()) { return false; }
  const auto values = sample();
  if (values == previous_) { return true; }
  if (message_.load(std::memory_order_acquire) != 0) {
    return fail("DSP published another input before the preceding validation");
  }
  previous_ = values;
  attained(ticks);
  input_ = {.ticks = ticks, .serial = ++serial_, .values = values};
  message_.store(serial_, std::memory_order_release);
  notify();
  while (!stopped() && message_.load(std::memory_order_acquire) != 0) {
    const auto observed = observation();
    if (message_.load(std::memory_order_acquire) != 0) { wait(observed); }
  }
  return !stopped();
}

bool Coupling::apply(const mcu::Effects::Entry &entry) {
  using Kind = mcu::Effects::Kind;
  switch (entry.kind) {
    case Kind::reset_hold:
      peer_.hold_reset();
      if (!reset_.assert_reset() || reset_.state().epoch != entry.value) {
        return fail("Staged DSP reset assertion diverged from its actual epoch");
      }
      return true;
    case Kind::reset_release:
      if (!reset_.quiescent()) { return false; }
      if (!peer_.release_mcu_data(entry.ticks, stop_)) { return false; }
      time_.release_dsp(entry.ticks);
      return reset_.release(stop_);
    case Kind::wait: time_.publish_mcu_wait(entry.ticks); return true;
    case Kind::acknowledge: return peer_.acknowledge_mcu(entry.value);
    case Kind::data:
    case Kind::strobe:
    case Kind::select:
    case Kind::word:
    case Kind::midi:
    default: return mcu::deliver(entry, peer_, shi_, midi_, stop_);
  }
}

bool Coupling::service(const Stamp stamp) {
  if (!bound_.load(std::memory_order_acquire)) { return true; }
  refresh();
  const auto exclusive = stamp.ceil();
  while (cursor_ < published_.load(std::memory_order_acquire) &&
         storage_[cursor_].ticks < exclusive && !stopped()) {
    const auto entry = storage_[cursor_];
    if (entry.kind == mcu::Effects::Kind::reset_release && !reset_.quiescent()) { break; }
    if (!apply(entry)) { return fail("Staged MCU output was refused by its actual receiver"); }
    ++cursor_;
    consumed_.store(cursor_, std::memory_order_release);
    cut_.store(exclusive, std::memory_order_release);
    if (!outputs(exclusive)) { return false; }
    if (reset_.state().held) {
      // Reset leaves native execution without going through receiver().
      attained(exclusive);
      break;
    }
  }
  return !stopped();
}

bool Coupling::checkpoint_barrier() {
  const auto command = checkpoint_request_.load(std::memory_order_acquire);
  if (command == 0) { return false; }
  const auto ticks = checkpoint_ticks_.load(std::memory_order_relaxed);
  if (known() < ticks || (known() == ticks && !reset_.state().held) ||
      checkpoint_request_.load(std::memory_order_acquire) != command) {
    return false;
  }
  cut_.store(known() + std::uint64_t{reset_.state().held}, std::memory_order_release);
  checkpoint_ready_.store(command, std::memory_order_release);
  notify();
  while (!stopped() && checkpoint_request_.load(std::memory_order_acquire) == command) {
    const auto observed = observation();
    if (checkpoint_request_.load(std::memory_order_acquire) == command) { wait(observed); }
  }
  return true;
}

Coupling::Grant Coupling::grant(const Stamp stamp) const noexcept {
  auto until = frontier_.load(std::memory_order_acquire);
  if (const auto input = midi_.input_limit()) { until = std::min(until, *input); }
  const auto count = published_.load(std::memory_order_acquire);
  const bool strobe = peer_.dsp_interrupt(false).has_value();
  for (auto index = cursor_; index < count; ++index) {
    const auto &entry = storage_[index];
    if (entry.ticks >= until) { break; }
    const bool ordinary =
        entry.kind == mcu::Effects::Kind::data || entry.kind == mcu::Effects::Kind::midi ||
        entry.kind == mcu::Effects::Kind::wait || entry.kind == mcu::Effects::Kind::acknowledge;
    if (ordinary || (entry.kind == mcu::Effects::Kind::strobe && (entry.value != 0) == strobe)) {
      continue;
    }
    until = entry.ticks;
    break;
  }
  if (until <= stamp.ticks) {
    return {.until = stamp.ticks + 1, .kind = Coupling::Grant::Kind::current};
  }
  return {.until = until};
}

std::optional<Coupling::Grant> Coupling::receiver(const Stamp stamp) {
  while (!stopped()) {
    const auto observed = observation();
    if (!service(stamp)) { return {}; }
    attained(stamp.ceil());
    if (reset_.state().held) { return {}; }
    if (checkpoint_barrier()) { continue; }
    const auto frontier = frontier_.load(std::memory_order_acquire);
    const auto input = midi_.input_limit();
    const bool midi_blocked = input && *input <= stamp.ticks;
    if (midi_blocked && midi_.audio_driven()) {
      if (!midi_.prepare_audio(stamp.ticks, stop_)) { return {}; }
      continue;
    }
    if (midi_blocked) { midi_.request_input(stamp.ceil()); }
    if (!midi_blocked && bound_.load(std::memory_order_acquire) &&
        (frontier > stamp.ticks || (frontier == stamp.ticks && !stamp.fractional))) {
      return grant(stamp);
    }
    if (!stopped() && frontier_.load(std::memory_order_acquire) == frontier &&
        generation_.load(std::memory_order_acquire) == receiver_generation_) {
      wait(observed);
    }
  }
  return {};
}

bool Coupling::service_held() {
  refresh();
  if (!bound_.load(std::memory_order_acquire)) { return true; }
  const auto frontier = frontier_.load(std::memory_order_acquire);
  while (cursor_ < published_.load(std::memory_order_acquire) && reset_.state().held) {
    const auto entry = storage_[cursor_];
    if (!apply(entry)) { return fail("MCU output failed while the DSP was held in reset"); }
    ++cursor_;
    consumed_.store(cursor_, std::memory_order_release);
    cut_.store(entry.ticks + 1, std::memory_order_release);
    if (!outputs(entry.ticks)) { return false; }
  }
  if (reset_.state().held) { attained(frontier); }
  return !stopped();
}

bool Coupling::held() {
  while (!stopped() && reset_.state().held) {
    const auto observed = observation();
    if (!service_held()) { return false; }
    if (!reset_.state().held) { return true; }
    if (checkpoint_barrier()) { continue; }
    if (!stopped()) { wait(observed); }
  }
  return !stopped();
}
}
