// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>

namespace blofeld {
// FM13-68/69,118/122: EOR data,A/B beside R0..R7 -> X/Y:absolute24.
[[nodiscard]] constexpr bool parallel_eor_address_store(const std::uint32_t opcode) {
  constexpr unsigned word_mask = 0xffffff;
  constexpr unsigned encoding_mask = 0xf0ffc7;
  constexpr unsigned eor_address_store = 0x607043;
  return opcode <= word_mask && (opcode & encoding_mask) == eor_address_store;
}
}
