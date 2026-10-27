// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"

namespace blofeld::detail::hcs08 {
constexpr unsigned decimal_opcode = 0x72;

// Family RM Rev2 Table A-2 specifies only these legal ADD/ADC result domains.
// This admission check supplies no arithmetic or architectural result.
inline void require_decimal_input(const Hcs08State &state) {
  constexpr unsigned half = 0x10;
  constexpr unsigned carry = 1;
  constexpr unsigned low_mask = 15;
  constexpr unsigned high_shift = 4;
  constexpr unsigned all = 0xff;
  const unsigned low = state.a & low_mask;
  const unsigned high = state.a >> high_shift;
  const bool h = (state.ccr & half) != 0;
  const bool c = (state.ccr & carry) != 0;
  const bool legal =
      (!c && (!h || low <= 3)) || (c && ((!h && high <= 2) || (h && high <= 3 && low <= 3)));
  if (state.a_known != all || (state.ccr_known & (half | carry)) != (half | carry) || !legal) {
    throw Fault("HCS08 DAA input address=" + hex(state.pc) + " pc=" + hex(state.pc) +
                " operation=decimal-adjust agent=CPU a=" + hex(state.a) +
                " a-known=" + hex(state.a_known) + " ccr=" + hex(state.ccr) +
                " ccr-known=" + hex(state.ccr_known) +
                ": requires known A/H/C and a legal Family RM Table A-2 input");
  }
}
}
