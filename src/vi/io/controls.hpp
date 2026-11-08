// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>

namespace blofeld::vi {
// External control levels, sampled by the MCU owner. The firmware applies pedal
// polarity and emits its own change event; no host-side MIDI/peer reply is added.
class Controls {
public:
  void pedal(const bool level) noexcept { pedal_.store(level, std::memory_order_release); }

  [[nodiscard]] bool pedal() const noexcept { return pedal_.load(std::memory_order_acquire); }

private:
  std::atomic<bool> pedal_{false};
};
}
