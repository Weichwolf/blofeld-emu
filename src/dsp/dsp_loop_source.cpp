// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_loop_source.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<DspLoopCountSource> automatic_loop_source(const std::uint32_t opcode) {
  constexpr unsigned word_mask = 0xffffff;
  if (opcode > word_mask) { return std::nullopt; }
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate_do = 0x060080;
  constexpr unsigned register_mask = 0xffc0ff;
  constexpr unsigned register_do = 0x06c000;
  constexpr unsigned source_mask = 0x3f;
  constexpr unsigned source_x0 = 4;
  constexpr unsigned source_r0 = 16;
  constexpr unsigned source_a = 14;
  constexpr unsigned source_a0 = 8;
  if ((opcode & immediate_mask) == immediate_do) { return DspLoopCountSource::immediate; }
  if ((opcode & register_mask) != register_do) { return std::nullopt; }
  switch ((opcode >> 8) & source_mask) {
    case source_x0: return DspLoopCountSource::x0;
    case source_r0: return DspLoopCountSource::r0;
    case source_a: return DspLoopCountSource::a;
    case source_a0: return DspLoopCountSource::a0;
    default: return std::nullopt;
  }
}
}
