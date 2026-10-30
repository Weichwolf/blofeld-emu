// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// FM13-136/Table12-13: full A/B sources, W=1, pp or split qq offsets.
[[nodiscard]] constexpr bool full_accumulator_peripheral_store(const std::uint32_t opcode) {
  constexpr unsigned high_mask = 0xfefec0;
  constexpr unsigned high_store = 0x08ce00;
  constexpr unsigned low_mask = 0xfffea0;
  constexpr unsigned low_x_store = 0x04ce80;
  constexpr unsigned low_y_store = 0x04ce20;
  constexpr unsigned word_mask = 0xffffff;
  return opcode <= word_mask &&
         ((opcode & high_mask) == high_store || (opcode & low_mask) == low_x_store ||
          (opcode & low_mask) == low_y_store);
}
}
