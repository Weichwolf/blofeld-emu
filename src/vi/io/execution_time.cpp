// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"

#include <atomic>
#include <cstdint>
#include <optional>

namespace blofeld::vi {
void ExecutionTime::publish_mcu_wait(const std::uint64_t ticks) noexcept {
  if (waited_.load(std::memory_order_relaxed)) { return; }
  first_wait_.store(ticks, std::memory_order_relaxed);
  waited_.store(true, std::memory_order_release);
}

std::optional<std::uint64_t> ExecutionTime::mcu_wait_time() const noexcept {
  if (!waited_.load(std::memory_order_acquire)) { return {}; }
  return first_wait_.load(std::memory_order_relaxed);
}

void ExecutionTime::release_dsp(const std::uint64_t origin) noexcept {
  origin_.store(origin, std::memory_order_release);
}

std::uint64_t ExecutionTime::dsp_origin() const noexcept {
  return origin_.load(std::memory_order_acquire);
}
}
