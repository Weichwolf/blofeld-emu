// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <string>

namespace blofeld::test {
inline constexpr unsigned transfer_address = 0x100;
inline constexpr unsigned transfer_second_address = 0x200;

inline void
set_accumulator(DspProgram &program, const std::string &name, const std::uint64_t value) {
  program.immediate(name + '0', value & word_mask);
  program.immediate(name + '1', (value >> word_bits) & word_mask);
  program.immediate(name + '2', value >> data_bits);
}

inline void run_to(DspCore &cpu, const unsigned pc) {
  unsigned count = 0;
  while (cpu.state().pc != pc) {
    constexpr unsigned instruction_limit = 100;
    require(++count <= instruction_limit, "Own DSP transfer diagnostic exceeded its setup limit");
    cpu.step();
  }
}
}
