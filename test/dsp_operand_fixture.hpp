// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp_norm_fixture.hpp"

#include <cstdint>

namespace blofeld::test::operands {
inline constexpr unsigned operand_preamble_steps = 10;
inline constexpr unsigned all_ccr = 0xff;

struct OperandInput {
  std::uint64_t a = 0;
  std::uint64_t b = 0;
  unsigned control = 0;
  unsigned source = 0;
  unsigned status = 0;
};

inline void seed_operands(DspProgram &code, const OperandInput &input) {
  code.immediate("a0", input.a & word_mask);
  code.immediate("a1", (input.a >> word_bits) & word_mask);
  code.immediate("a2", input.a >> data_bits);
  code.immediate("b0", input.b & word_mask);
  code.immediate("b1", (input.b >> word_bits) & word_mask);
  code.immediate("b2", input.b >> data_bits);
  code.immediate("x0", input.control);
  code.immediate("x1", input.source);
  code.immediate("y1", input.source ^ word_mask);
  code.immediate("sr", masked_status | input.status);
}

void logical_checks();
void bitfield_checks();
void rejection_checks();
}
