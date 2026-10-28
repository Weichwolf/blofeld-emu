// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_memory_form.hpp"

#include <cstdint>

namespace blofeld {
inline constexpr std::uint32_t dsp_repeat_from_a1 = 0x06cc20;

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
  // FM13-118/122,Table12-14: ordinary scalar stores,modes0..5/7.
  constexpr unsigned mode_shift = 11;
  constexpr unsigned mode_mask = 7;
  constexpr unsigned extended_mode = 6;
  const auto mode = (opcode >> mode_shift) & mode_mask;
  return (full_accumulator_scalar_store(opcode) || partial_accumulator_scalar_store(opcode) ||
          data_offset_scalar_store(opcode)) &&
         mode != extended_mode;
}

// FM13-160/Table12-13: immediate twelve-bit integer,raw X0 or raw A1.
inline bool fetch_repeat_prefix(const std::uint32_t opcode) {
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate = 0x0600a0;
  constexpr unsigned from_x0 = 0x06c420;
  return (opcode & immediate_mask) == immediate || opcode == from_x0 ||
         opcode == dsp_repeat_from_a1;
}

// The caller reads only the selected native count register at actual E1.
struct DspRepeatCountInputs {
  std::uint32_t opcode = 0, register_count = 0;
};

inline std::uint32_t fetch_repeat_count(const DspRepeatCountInputs inputs) {
  constexpr unsigned from_x0 = 0x06c420;
  constexpr unsigned byte_mask = 0xff;
  if (inputs.opcode == from_x0 || inputs.opcode == dsp_repeat_from_a1) {
    return inputs.register_count;
  }
  return ((inputs.opcode >> 8) & byte_mask) | ((inputs.opcode & 0xfU) << 8);
}
}
