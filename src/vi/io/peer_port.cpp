// SPDX-License-Identifier: GPL-3.0-only
#include "io/peer_port.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
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
}

PeerPort::Sample PeerPort::mcu_data() const noexcept {
  return decode(mcu_.load(std::memory_order_acquire));
}

PeerPort::Sample PeerPort::dsp_data() const noexcept {
  return decode(dsp_.load(std::memory_order_acquire));
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
  return to_dsp_.update(active);
}

bool PeerPort::dsp_ready(const bool active) noexcept {
  return to_mcu_.update(active);
}

std::optional<std::uint64_t> PeerPort::dsp_interrupt(const bool edge) const noexcept {
  return to_dsp_.pending(edge);
}

std::optional<std::uint64_t> PeerPort::mcu_interrupt() const noexcept {
  return to_mcu_.pending(true);
}

bool PeerPort::acknowledge_dsp(const std::uint64_t observed) noexcept {
  return to_dsp_.acknowledge(observed);
}

bool PeerPort::acknowledge_mcu(const std::uint64_t observed) noexcept {
  return to_mcu_.acknowledge(observed);
}

void PeerPort::reset_dsp() noexcept {
  dsp_data({});
  to_mcu_.active.store(false, std::memory_order_release);
  static_cast<void>(to_mcu_.acknowledge(to_mcu_.posted.load(std::memory_order_acquire)));
  static_cast<void>(to_dsp_.acknowledge(to_dsp_.posted.load(std::memory_order_acquire)));
}
}
