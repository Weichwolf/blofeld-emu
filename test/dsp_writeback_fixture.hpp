// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <cstdint>
#include <variant>

namespace blofeld::test::writeback {
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;
inline constexpr unsigned source = 0x100;
inline constexpr unsigned sample = 0x800001;
inline constexpr std::uint64_t seed_x = 0x112233445566;
inline constexpr std::uint64_t seed_y = 0x223344556677;
inline constexpr std::uint64_t seed_a = 0x123456789abcde;
inline constexpr std::uint64_t seed_b = 0x23456789abcdef;

inline DspMoveWriteback move(const DspReferenceProgress &progress) {
  if (!progress.pending || progress.retired) {
    throw Fault("Expected pending native move writeback");
  }
  const auto *value = std::get_if<DspMoveWriteback>(&progress.pending->effect);
  if (value == nullptr || progress.pending->execution != execution) {
    throw Fault("Expected the explicit staged-register execution contract");
  }
  return *value;
}

inline DspReferenceProgress await_move(DspMachine &machine, DspReferenceProgress progress) {
  constexpr unsigned limit = 16;
  for (unsigned count = 0; count < limit; ++count) {
    if (progress.pending && std::holds_alternative<DspMoveWriteback>(progress.pending->effect)) {
      return progress;
    }
    if (progress.retired || !progress.pending ||
        std::holds_alternative<DspInstructionRetire>(progress.pending->effect)) {
      throw Fault("Own instruction did not reach a genuine move-writeback boundary");
    }
    progress = machine.advance_reference_access(reference::ticket(progress));
  }
  throw Fault("Own move-writeback effect budget exhausted");
}

inline DspReferenceProgress retire(DspMachine &machine, const DspReferenceProgress &progress) {
  require(progress.pending &&
              std::holds_alternative<DspInstructionRetire>(progress.pending->effect),
          "The actual remaining CPU result must await its later native retirement");
  return machine.advance_reference_access(reference::ticket(progress));
}

inline void seed(DspProgram &code) {
  code.immediate("x0", seed_x & word_mask);
  code.immediate("x1", seed_x >> word_bits);
  code.immediate("y0", seed_y & word_mask);
  code.immediate("y1", seed_y >> word_bits);
  code.immediate("a0", seed_a & word_mask);
  code.immediate("a1", (seed_a >> word_bits) & word_mask);
  code.immediate("a2", seed_a >> data_bits);
  code.immediate("b0", seed_b & word_mask);
  code.immediate("b1", (seed_b >> word_bits) & word_mask);
  code.immediate("b2", seed_b >> data_bits);
  code.immediate("sr", masked_status | carry | negative);
}

void parallel_checks();
void clock_checks();
}
