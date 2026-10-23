// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
enum class McuCopReset : std::uint8_t { none, timeout, invalid_service, early_service };

struct McuCopState {
  std::uint8_t control = 0;
  unsigned phase = 0;
  bool control_written = false;
  bool armed = false;
  McuCopReset reset = McuCopReset::none;
  bool operator==(const McuCopState &) const = default;
};

// S12CRGUV1 normal-user-mode watchdog. OSCCLK edges are explicit inputs.
// A request is latched at its first cause; applying the system reset is separate.
class McuCop {
public:
  void reset();
  void write_control(std::uint8_t value);
  void service(std::uint8_t value);
  // Return elapsed edges, stopping at the first reset request without wrapping.
  unsigned advance(unsigned cycles);
  [[nodiscard]] unsigned period() const;
  [[nodiscard]] unsigned remaining() const;

  [[nodiscard]] McuCopState state() const { return state_; }

private:
  void healthy() const;
  McuCopState state_;
};
}
