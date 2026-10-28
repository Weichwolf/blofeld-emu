// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_memory_form.hpp"

#include <cstdint>

namespace blofeld {
// Actual native control projection, never authority to write architectural state.
struct DspRepeatControl {
  std::uint32_t pc = 0, omr = 0, lc = 0;
  std::uint32_t remaining = 0, body = 0, opcode = 0, extension = 0, next = 0, saved_lc = 0;
  bool loaded = false;
  bool operator==(const DspRepeatControl &) const = default;
};

inline bool fetch_repeat_body(const std::uint32_t opcode) {
  constexpr unsigned store_a_long_r4 = 0x485c00;
  if (opcode == 0 || opcode == store_a_long_r4) { return true; }
  // FM13-118/122,Table12-14: standalone full/partial accumulator stores,modes0..4.
  constexpr unsigned mode_shift = 11;
  constexpr unsigned mode_mask = 7;
  return (full_accumulator_scalar_store(opcode) || partial_accumulator_scalar_store(opcode)) &&
         ((opcode >> mode_shift) & mode_mask) <= 4;
}

// FM13-160: immediate twelve-bit integer or the complete X0 register.
inline bool fetch_repeat_prefix(const std::uint32_t opcode) {
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate = 0x0600a0;
  constexpr unsigned from_x0 = 0x06c420;
  return (opcode & immediate_mask) == immediate || opcode == from_x0;
}

inline std::uint32_t fetch_repeat_count(const std::uint32_t opcode, const std::uint32_t x0) {
  constexpr unsigned from_x0 = 0x06c420;
  constexpr unsigned byte_mask = 0xff;
  return opcode == from_x0 ? x0 : ((opcode >> 8) & byte_mask) | ((opcode & 0xfU) << 8);
}
}
