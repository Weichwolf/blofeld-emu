// SPDX-License-Identifier: GPL-3.0-only
#include "io/peer_port.hpp"

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
namespace {
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
constexpr std::uint64_t admitting = 1, notified = 2, admission_flags = 3, next_admission = 4;

std::uint32_t encode(const PeerPort::Sample value) noexcept {
  return (std::uint32_t{value.mask} << 16U) | (std::uint32_t{value.known} << 8U) | value.value;
}

PeerPort::Sample decode(const std::uint32_t word) noexcept {
  constexpr std::uint32_t byte_mask = 0xff;
  return {.mask = static_cast<std::uint8_t>(word >> 16U),
          .known = static_cast<std::uint8_t>((word >> 8U) & byte_mask),
          .value = static_cast<std::uint8_t>(word & byte_mask)};
}
}

bool PeerPort::wait_input_space(const std::uint64_t published, const std::stop_token &stop) {
  inputs_.waiting.store(true, std::memory_order_seq_cst);
  const std::stop_callback cancelled(stop, [this] {
    inputs_.changed.fetch_add(1, std::memory_order_seq_cst);
    inputs_.changed.notify_one();
  });
  bool available = false;
  while (!stop.stop_requested()) {
    const auto observed = inputs_.changed.load(std::memory_order_seq_cst);
    if (published - inputs_.consumed.load(std::memory_order_seq_cst) < Inputs::capacity) {
      available = true;
      break;
    }
#if defined(BLOFELD_CENSUS)
    const Census::Wait observed_wait;
#endif
    inputs_.changed.wait(observed, std::memory_order_acquire);
  }
  inputs_.waiting.store(false, std::memory_order_seq_cst);
  return available;
}

bool PeerPort::mcu_data(const Sample value,
                        const std::uint64_t ticks,
                        const std::stop_token &stop) {
  if (ticks < inputs_.last_tick) { return fail("MCU peer data time moved backwards"); }
  if (stop.stop_requested()) { return false; }
  const auto word = encode(value);
  if (inputs_.held) {
    mcu_.store(word, std::memory_order_release);
  } else if (mcu_.load(std::memory_order_relaxed) != word) {
    if (!append_mcu_data(value, ticks, stop)) { return false; }
  }
  inputs_.last_tick = ticks;
  return true;
}

bool PeerPort::append_mcu_data(const Sample value,
                               const std::uint64_t ticks,
                               const std::stop_token &stop) {
  const auto published = inputs_.published.load(std::memory_order_relaxed);
  if (published == std::numeric_limits<std::uint64_t>::max()) {
    return fail("MCU peer data publication count is exhausted");
  }
  if (published - inputs_.consumed.load(std::memory_order_acquire) == Inputs::capacity &&
      !wait_input_space(published, stop)) {
    return false;
  }
  inputs_.entries[published % Inputs::capacity] = {.ticks = ticks, .data = value};
  inputs_.published.store(published + 1, std::memory_order_release);
  mcu_.store(encode(value), std::memory_order_release);
  return true;
}

void PeerPort::release_input_space(const std::uint64_t consumed) noexcept {
  // SC registration/recheck makes a full-queue waiter observe space or a wake.
  inputs_.consumed.store(consumed, std::memory_order_seq_cst);
  if (inputs_.waiting.load(std::memory_order_seq_cst)) {
    inputs_.changed.fetch_add(1, std::memory_order_seq_cst);
    inputs_.changed.notify_one();
  }
}

PeerPort::Sample PeerPort::sample_mcu_data(const std::uint64_t ticks) noexcept {
  auto consumed = inputs_.consumed.load(std::memory_order_relaxed);
  const auto before = consumed;
  const auto published = inputs_.published.load(std::memory_order_acquire);
  while (consumed != published) {
    const auto &input = inputs_.entries[consumed % Inputs::capacity];
    if (input.ticks > ticks) { break; }
    inputs_.sample = input.data;
    ++consumed;
  }
  if (consumed != before) { release_input_space(consumed); }
  return inputs_.sample;
}

void PeerPort::hold_mcu_data() noexcept {
  inputs_.held = true;
  inputs_.reset_through.store(inputs_.published.load(std::memory_order_relaxed),
                              std::memory_order_release);
}

bool PeerPort::release_mcu_data(const std::uint64_t ticks, const std::stop_token &stop) {
  if (!inputs_.held) { return true; }
  if (ticks < inputs_.last_tick) { return fail("MCU peer reset release time moved backwards"); }
  if (stop.stop_requested() || !append_mcu_data(mcu_data(), ticks, stop)) { return false; }
  inputs_.last_tick = ticks;
  inputs_.held = false;
  return true;
}

void PeerPort::retire_mcu_data() noexcept {
  const auto through = inputs_.reset_through.load(std::memory_order_acquire);
  if (through > inputs_.consumed.load(std::memory_order_relaxed)) { release_input_space(through); }
}

void PeerPort::dsp_data(const Sample value) noexcept {
  const auto word = encode(value);
  if (dsp_.load(std::memory_order_relaxed) != word) {
    reply_.announced.store(false, std::memory_order_release);
    dsp_.store(word, std::memory_order_release);
  }
  constexpr std::uint8_t byte_mask = 0xff;
  if (value.mask != byte_mask && reply_.recording.load(std::memory_order_acquire)) {
    const std::lock_guard lock(reply_.mutex);
    reply_.recording.store(false, std::memory_order_release);
    if (reply_.written != reply_.bytes.size()) {
      static_cast<void>(fail("DSP released an incomplete peer response block"));
    }
  }
}

PeerPort::Sample PeerPort::mcu_data() const noexcept {
  return decode(mcu_.load(std::memory_order_acquire));
}

PeerPort::Sample PeerPort::dsp_data() const noexcept {
  return decode(dsp_.load(std::memory_order_acquire));
}

bool PeerPort::fail(const char *message) noexcept {
  error_.store(message, std::memory_order_release);
  return false;
}

bool PeerPort::start_reply() noexcept {
  const std::lock_guard lock(reply_.mutex);
  if (reply_.active.load(std::memory_order_relaxed) ||
      reply_.recording.load(std::memory_order_relaxed)) {
    return fail("MCU requested a peer response before completing the preceding block");
  }
  reply_.written = reply_.read = 0;
  reply_.active.store(true, std::memory_order_release);
  reply_.recording.store(true, std::memory_order_release);
  return true;
}

bool PeerPort::dsp_write(const Sample value, const std::uint64_t ticks) noexcept {
  if (!reply_.recording.load(std::memory_order_acquire)) { return true; }
  const std::lock_guard lock(reply_.mutex);
  if (!reply_.recording.load(std::memory_order_relaxed)) { return true; }
  if (reply_.written == reply_.bytes.size()) { return fail("DSP peer response block overflow"); }
  if (reply_.written != 0 && ticks < reply_.bytes[reply_.written - 1].ticks) {
    return fail("DSP peer response time moved backwards");
  }
  reply_.bytes[reply_.written++] = {.ticks = ticks, .data = value};
  return true;
}

std::optional<PeerPort::Sample> PeerPort::read_dsp_data(const Read access) noexcept {
  constexpr std::uint8_t byte_mask = 0xff;
  if (access.mask != byte_mask || mcu_data().mask != 0 ||
      !reply_.active.load(std::memory_order_acquire)) {
    return dsp_data();
  }
  const std::lock_guard lock(reply_.mutex);
  if (!reply_.active.load(std::memory_order_relaxed)) { return dsp_data(); }
  if (reply_.read == reply_.written) {
    static_cast<void>(fail("MCU read a peer response byte before the DSP produced it"));
    return {};
  }
  const auto &byte = reply_.bytes[reply_.read];
  if (byte.ticks > access.ticks) {
    static_cast<void>(fail("MCU read a peer response byte before its DSP instrument time"));
    return {};
  }
  const auto result = byte.data;
  ++reply_.read;
  if (reply_.read == reply_.bytes.size()) { reply_.active.store(false, std::memory_order_release); }
  return result;
}

bool PeerPort::Request::update(const bool value) noexcept {
  const auto previous = active.load(std::memory_order_relaxed);
  if (value == previous) { return true; }
  if (value) {
    const auto count = posted.load(std::memory_order_relaxed);
    if (count == std::numeric_limits<std::uint64_t>::max()) { return false; }
    active.store(value, std::memory_order_release);
    posted.store(count + 1, std::memory_order_release);
  } else {
    active.store(value, std::memory_order_release);
  }
  return true;
}

std::optional<std::uint64_t> PeerPort::Request::pending(const bool edge) const noexcept {
  const auto count = posted.load(std::memory_order_acquire);
  if (!edge) {
    return active.load(std::memory_order_acquire) ? std::optional{count} : std::nullopt;
  }
  return count > accepted.load(std::memory_order_relaxed) ? std::optional{count} : std::nullopt;
}

bool PeerPort::Request::acknowledge(const std::uint64_t observed) noexcept {
  if (observed > posted.load(std::memory_order_acquire)) { return false; }
  auto before = accepted.load(std::memory_order_relaxed);
  while (before < observed &&
         !accepted.compare_exchange_weak(
             before, observed, std::memory_order_release, std::memory_order_relaxed)) {}
  return true;
}

bool PeerPort::mcu_interrupt_ready(const bool ready) noexcept {
  const auto before = mcu_admission_.load(std::memory_order_acquire);
  if (((before & admitting) != 0) == ready) { return true; }
  const auto generation = before & ~admission_flags;
  if (generation > std::numeric_limits<std::uint64_t>::max() - next_admission) {
    return fail("MCU peer interrupt admission generation is exhausted");
  }
  // A mask transition revokes pending transport notifications. DSP publication
  // uses CAS on this exact window, so an old eligibility observation cannot
  // survive a mask/unmask cycle (including an ABA change of the ready bit).
  mcu_admission_.store(generation + next_admission + (ready ? admitting : 0),
                       std::memory_order_release);
  return true;
}

bool PeerPort::mcu_strobe(const bool active) noexcept {
  if (active && !to_dsp_.active.load(std::memory_order_relaxed) &&
      reply_.announced.exchange(false, std::memory_order_acq_rel) && !start_reply()) {
    return false;
  }
  return to_dsp_.update(active);
}

bool PeerPort::dsp_ready(const bool active) noexcept {
  const bool posted = active && !to_mcu_.active.load(std::memory_order_relaxed);
  auto admission = posted ? mcu_admission_.load(std::memory_order_acquire) : 0;
  if (posted) {
    constexpr std::uint8_t byte_mask = 0xff;
    constexpr std::uint8_t command_mask = 0x3f;
    constexpr std::uint8_t report = 0x0d;
    constexpr std::uint8_t bulk = 0x0e;
    const auto data = dsp_data();
    const auto command = data.value & command_mask;
    if (data.mask == byte_mask && data.known == byte_mask &&
        (command == report || command == bulk)) {
      reply_.announced.store(true, std::memory_order_release);
    }
  }
  if (!to_mcu_.update(active)) { return false; }
  if (posted && (admission & admitting) != 0) {
    // A real producer event, not a fabricated MCU acknowledgement. Native
    // interrupt acceptance still owns the posted/accepted request generations.
    static_cast<void>(mcu_admission_.compare_exchange_strong(
        admission, admission | notified, std::memory_order_acq_rel, std::memory_order_acquire));
  }
  return true;
}

std::optional<std::uint64_t> PeerPort::dsp_interrupt(const bool edge) const noexcept {
  return to_dsp_.pending(edge);
}

std::optional<std::uint64_t> PeerPort::mcu_interrupt() const noexcept {
  if (const auto level = to_mcu_.pending(false)) { return level; }
  if ((mcu_admission_.load(std::memory_order_acquire) & admission_flags) == admission_flags) {
    if (const auto notification = to_mcu_.pending(true)) { return notification; }
  }
  if (reply_.announced.load(std::memory_order_acquire)) {
    return to_mcu_.posted.load(std::memory_order_acquire);
  }
  return {};
}

bool PeerPort::acknowledge_dsp(const std::uint64_t observed) noexcept {
  return to_dsp_.acknowledge(observed);
}

bool PeerPort::acknowledge_mcu(const std::uint64_t observed) noexcept {
  return to_mcu_.acknowledge(observed);
}

void PeerPort::reset_dsp() noexcept {
  {
    const std::lock_guard lock(reply_.mutex);
    reply_.active.store(false, std::memory_order_release);
    reply_.recording.store(false, std::memory_order_release);
    reply_.announced.store(false, std::memory_order_release);
    reply_.read = reply_.written = 0;
    error_.store(nullptr, std::memory_order_release);
  }
  dsp_data({});
  mcu_admission_.fetch_and(~notified, std::memory_order_acq_rel);
  to_mcu_.active.store(false, std::memory_order_release);
  static_cast<void>(to_mcu_.acknowledge(to_mcu_.posted.load(std::memory_order_acquire)));
  static_cast<void>(to_dsp_.acknowledge(to_dsp_.posted.load(std::memory_order_acquire)));
}
}
