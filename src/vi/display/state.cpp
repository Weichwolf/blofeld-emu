// SPDX-License-Identifier: GPL-3.0-only
#include "display/state.hpp"

#include "display/st7565.hpp"

#include <atomic>
#include <mutex>
#include <optional>
#include <utility>

namespace blofeld::vi {
void DisplayState::capture(const St7565 &controller) {
  if (!requested_.load(std::memory_order_relaxed) ||
      !requested_.exchange(false, std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard lock(mutex_);
  pending_ = controller;
}

std::optional<St7565> DisplayState::take() {
  const std::lock_guard lock(mutex_);
  return std::exchange(pending_, std::nullopt);
}
}
