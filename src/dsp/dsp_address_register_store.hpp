// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// Raw R0..R7 scalar stores; separate from data/offset and REP qualification.
[[nodiscard]] constexpr bool address_register_scalar_store(const std::uint32_t opcode) {
  // FM13-118/122,Table12-14: fields16..23, W=0, EA=1, no parallel ALU.
  constexpr unsigned word_mask = 0xffffff;
  constexpr unsigned encoding_mask = 0xf0c0ff;
  constexpr unsigned address_store = 0x604000;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned mode_mask = 7;
  constexpr unsigned extended = 6;
  return opcode <= word_mask && (opcode & encoding_mask) == address_store &&
         ((opcode >> mode_shift) & mode_mask) != extended;
}

}
