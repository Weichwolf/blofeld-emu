// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"
#include "mcu/ports.hpp"

#include <cstdint>

namespace blofeld::vi {
// Decode the firmware's A/B/E register transactions, without pad/timing replay.
// E4 latch writes delimit accesses even when the firmware releases DDRE.
class DisplayPort {
public:
  DisplayPort();
  [[nodiscard]] bool write(const mcu::Ports &ports, mcu::Ports::Write access) noexcept;
  [[nodiscard]] bool read(std::uint8_t &value, std::uint8_t mask) const noexcept;

  [[nodiscard]] bool reading() const noexcept { return active_ && reading_; }

  [[nodiscard]] const St7565 &controller() const noexcept { return controller_; }

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  [[nodiscard]] bool update(const mcu::Ports &ports, mcu::Ports::Write access);
  [[nodiscard]] bool finish(const mcu::Ports &ports);
  St7565 controller_;
  LcdByte held_;
  const char *error_ = nullptr;
  bool enable_ = false, active_ = false, reading_ = false, data_ = false;
};
}
