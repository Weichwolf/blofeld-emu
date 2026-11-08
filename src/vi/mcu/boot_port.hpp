// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/shi.hpp"

#include <cstdint>
#include <optional>

namespace blofeld::vi::mcu {
// Port B firmware adapter: B1 selects, B2 clocks, B4 carries serial data.
// Collect locally; publish completed words to SHI without per-bit shared work.
class BootPort {
public:
  struct Write {
    std::uint16_t offset = 0;
    std::uint8_t value = 0;
  };

  explicit BootPort(Shi &shi) : shi_(shi) {}

  [[nodiscard]] bool read(std::uint16_t offset, std::uint8_t &value) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;

private:
  [[nodiscard]] bool update(std::uint8_t previous) noexcept;
  Shi &shi_;
  std::uint8_t latch_ = 0, direction_ = 0;
  bool latch_known_ = false, selected_ = false;
  std::optional<Shi::Mode> mode_;
  unsigned bits_ = 0;
  std::uint32_t word_ = 0;
};
}
