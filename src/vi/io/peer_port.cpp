// SPDX-License-Identifier: GPL-3.0-only
#include "io/peer_port.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>

namespace blofeld::vi {
namespace {
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

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

void PeerPort::mcu_data(const Sample value) noexcept {
  const auto word = encode(value);
  if (mcu_.load(std::memory_order_relaxed) != word) { mcu_.store(word, std::memory_order_release); }
}

void PeerPort::dsp_data(const Sample value) noexcept {
  const auto word = encode(value);
  if (dsp_.load(std::memory_order_relaxed) != word) { dsp_.store(word, std::memory_order_release); }
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

bool PeerPort::dsp_write(const Sample value) noexcept {
  if (!reply_.recording.load(std::memory_order_acquire)) { return true; }
  const std::lock_guard lock(reply_.mutex);
  if (!reply_.recording.load(std::memory_order_relaxed)) { return true; }
  if (reply_.written == reply_.bytes.size()) { return fail("DSP peer response block overflow"); }
  reply_.bytes[reply_.written++] = value;
  return true;
}

std::optional<PeerPort::Sample> PeerPort::read_dsp_data(const std::uint8_t mask) noexcept {
  constexpr std::uint8_t byte_mask = 0xff;
  if (mask != byte_mask || mcu_data().mask != 0 || !reply_.active.load(std::memory_order_acquire)) {
    return dsp_data();
  }
  const std::lock_guard lock(reply_.mutex);
  if (!reply_.active.load(std::memory_order_relaxed)) { return dsp_data(); }
  if (reply_.read == reply_.written) {
    static_cast<void>(fail("MCU read a peer response byte before the DSP produced it"));
    return {};
  }
  const auto result = reply_.bytes[reply_.read++];
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

bool PeerPort::mcu_strobe(const bool active) noexcept {
  if (active && !to_dsp_.active.load(std::memory_order_relaxed) &&
      reply_.announced.exchange(false, std::memory_order_acq_rel) && !start_reply()) {
    return false;
  }
  return to_dsp_.update(active);
}

bool PeerPort::dsp_ready(const bool active) noexcept {
  if (active && !to_mcu_.active.load(std::memory_order_relaxed)) {
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
  return to_mcu_.update(active);
}

std::optional<std::uint64_t> PeerPort::dsp_interrupt(const bool edge) const noexcept {
  return to_dsp_.pending(edge);
}

std::optional<std::uint64_t> PeerPort::mcu_interrupt() const noexcept {
  return to_mcu_.pending(false);
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
  to_mcu_.active.store(false, std::memory_order_release);
  static_cast<void>(to_mcu_.acknowledge(to_mcu_.posted.load(std::memory_order_acquire)));
  static_cast<void>(to_dsp_.acknowledge(to_dsp_.posted.load(std::memory_order_acquire)));
}
}
