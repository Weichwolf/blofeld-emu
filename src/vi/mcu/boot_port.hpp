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
    std::uint8_t direction = 0, known = 0, value = 0;
  };

  explicit BootPort(Shi &shi) : shi_(shi) {}

  [[nodiscard]] bool write(Write access) noexcept;
  void reset() noexcept;

private:
  Shi &shi_;
  std::uint8_t previous_ = 0;
  bool selected_ = false;
  std::optional<Shi::Mode> mode_;
  unsigned bits_ = 0;
  std::uint32_t word_ = 0;
};
}
