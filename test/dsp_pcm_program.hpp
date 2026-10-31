// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp_rom_program.hpp"

#include <array>

namespace blofeld::test {
inline DspRomProgram dsp_pcm_program(const bool secondary) {
  // Same authored81-word program as the qualified esai0/1-refill ROM fixtures.
  // Six genuine immediate MOVEPs configure the mux, clocks and transmitter.
  constexpr std::array primary{0x07f435U,
                               0x7d00U,
                               0x07f436U,
                               0x640301U,
                               0x07f43fU,
                               0x838U,
                               0x07f43eU,
                               0x838U,
                               0x07f420U,
                               0xa5c39fU,
                               0x07f435U,
                               0x7d01U};
  constexpr std::array second{0x07b495U,
                              0x7d00U,
                              0x07b496U,
                              0x640301U,
                              0x07b49fU,
                              0x838U,
                              0x07b49eU,
                              0x838U,
                              0x07b480U,
                              0xa5c39fU,
                              0x07b495U,
                              0x7d01U};
  DspRomProgram program;
  program.append(secondary ? second : primary);
  constexpr unsigned delay = 64;
  for (unsigned index = 0; index < delay; ++index) { program.word(0); }
  constexpr unsigned primary_refill = 0x07f420;
  constexpr unsigned secondary_refill = 0x07b480;
  constexpr unsigned next_word = 0x123456;
  program.append(std::array{secondary ? secondary_refill : primary_refill, next_word});
  const auto loop = program.pc();
  constexpr unsigned jump = 0x0af080;
  program.append(std::array{0U, jump, loop});
  return program;
}

}
