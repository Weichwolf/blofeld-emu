// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// Actual native control projection, never authority to write architectural state.
struct DspRepeatControl {
  std::uint32_t pc = 0, omr = 0, lc = 0;
  std::uint32_t remaining = 0, body = 0, opcode = 0, extension = 0, next = 0, saved_lc = 0;
  bool loaded = false;
  bool operator==(const DspRepeatControl &) const = default;
};

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
