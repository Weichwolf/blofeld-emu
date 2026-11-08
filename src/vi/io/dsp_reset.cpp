// SPDX-License-Identifier: GPL-3.0-only
#include "io/dsp_reset.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
DspReset::State DspReset::state() const noexcept {
  const auto value = state_.load(std::memory_order_acquire);
  return {.epoch = value >> 1U, .held = (value & 1) != 0};
}

bool DspReset::pending(const std::uint64_t epoch) const noexcept {
  const auto value = state_.load(std::memory_order_acquire);
  return (value >> 1U) != epoch || (value & 1) != 0;
}

bool DspReset::assert_reset() noexcept {
  const std::lock_guard lock(mutex_);
  const auto value = state_.load(std::memory_order_relaxed);
  if ((value & 1) != 0) { return true; }
  if (value > std::numeric_limits<std::uint64_t>::max() - 3) { return false; }
  state_.store(value + 3, std::memory_order_release);
  changed_.notify_all();
  return true;
}

bool DspReset::acknowledge(const std::uint64_t epoch) noexcept {
  const std::lock_guard lock(mutex_);
  if (epoch < acknowledged_ || epoch > state().epoch) { return false; }
  acknowledged_ = epoch;
  changed_.notify_all();
  return true;
}

bool DspReset::release(const std::stop_token &stop) {
  std::unique_lock lock(mutex_);
  if (!changed_.wait(lock, stop, [&] { return acknowledged_ == state().epoch; })) { return false; }
  if (stop.stop_requested()) { return false; }
  const auto value = state_.load(std::memory_order_relaxed);
  state_.store(value & ~std::uint64_t{1}, std::memory_order_release);
  changed_.notify_all();
  return true;
}

std::optional<DspReset::State> DspReset::wait_release(const std::stop_token &stop) {
  std::unique_lock lock(mutex_);
  if (!changed_.wait(lock, stop, [&] { return !state().held; }) || stop.stop_requested()) {
    return {};
  }
  return state();
}
}
