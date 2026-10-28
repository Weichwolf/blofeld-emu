// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_loop_source.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr unsigned y1_dor = 0x06c710; // FM13-61 register DDDDDD=7 selects Y1.
}

bool relative_loop(const std::uint32_t opcode) {
  // FM13-61: relative immediate or Y1 forms qualified by retained NXP windows.
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate_dor = 0x060090;
  constexpr unsigned word_mask = 0xffffff;
  return opcode <= word_mask && ((opcode & immediate_mask) == immediate_dor || opcode == y1_dor);
}

std::optional<DspLoopCountSource> automatic_loop_source(const std::uint32_t opcode) {
  constexpr unsigned word_mask = 0xffffff;
  if (opcode > word_mask) { return std::nullopt; }
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate_do = 0x060080;
  constexpr unsigned register_mask = 0xffc0ff;
  constexpr unsigned register_do = 0x06c000;
  constexpr unsigned source_mask = 0x3f;
  constexpr unsigned source_x0 = 4;
  constexpr unsigned source_y1 = 7;
  constexpr unsigned source_r0 = 16;
  constexpr unsigned source_a = 14;
  constexpr unsigned source_a0 = 8;
  if (relative_loop(opcode)) {
    return opcode == y1_dor ? DspLoopCountSource::y1 : DspLoopCountSource::immediate;
  }
  if ((opcode & immediate_mask) == immediate_do) { return DspLoopCountSource::immediate; }
  if ((opcode & register_mask) != register_do) { return std::nullopt; }
  switch ((opcode >> 8) & source_mask) {
    case source_x0: return DspLoopCountSource::x0;
    case source_y1: return DspLoopCountSource::y1;
    case source_r0: return DspLoopCountSource::r0;
    case source_a: return DspLoopCountSource::a;
    case source_a0: return DspLoopCountSource::a0;
    default: return std::nullopt;
  }
}
}
