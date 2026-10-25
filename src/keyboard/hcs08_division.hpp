// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "keyboard/hcs08.hpp"

namespace blofeld::detail::hcs08 {
inline constexpr unsigned divide_opcode = 0x52;

// RM Rev.2 p250: H:A / X fits an eight-bit quotient exactly when H < X.
// This also excludes X=0 without evaluating an invalid host division.
[[nodiscard]] constexpr bool division_defined(const Hcs08State &state) {
  constexpr unsigned byte_mask = 0xff;
  return (state.hx >> 8U) < (state.hx & byte_mask);
}
} // namespace blofeld::detail::hcs08
