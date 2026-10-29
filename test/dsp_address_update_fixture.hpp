// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <string>

namespace blofeld::test::address_update {
inline constexpr unsigned entry = 0x100;
inline constexpr unsigned ordinary_omr = 0x305;
inline constexpr unsigned status_without_carry = 0xc003fe;
// FM13-117:00100000010MMRRR00000000; MM=0..3 is -N,+N,-1,+1.
inline constexpr unsigned update_base = 0x204000;
inline constexpr unsigned mode_shift = 11;
inline constexpr unsigned register_shift = 8;
inline constexpr unsigned absolute_jump = 0x0af080;
inline constexpr unsigned padding = 32;

struct Case {
  unsigned r = 0, n = 0, m = 0;
  unsigned expected = 0, mode = 0;
};

// Independent FM4 arithmetic, also checked in192 pinned vendor-model windows.
// Rows are per mode because endpoint input depends on update direction.
inline constexpr std::array cases{
    // linear
    Case{.r = 0x0, .n = 0x7, .m = 0xffffff, .expected = 0xfffff9, .mode = 0},
    Case{.r = 0xffffff, .n = 0x7, .m = 0xffffff, .expected = 0x6, .mode = 1},
    Case{.r = 0x0, .n = 0x7, .m = 0xffffff, .expected = 0xffffff, .mode = 2},
    Case{.r = 0xffffff, .n = 0x7, .m = 0xffffff, .expected = 0x0, .mode = 3},
    // negative-offset
    Case{.r = 0x2, .n = 0xfffffd, .m = 0xffffff, .expected = 0x5, .mode = 0},
    Case{.r = 0x2, .n = 0xfffffd, .m = 0xffffff, .expected = 0xffffff, .mode = 1},
    Case{.r = 0x2, .n = 0xfffffd, .m = 0xffffff, .expected = 0x1, .mode = 2},
    Case{.r = 0x2, .n = 0xfffffd, .m = 0xffffff, .expected = 0x3, .mode = 3},
    // modulo
    Case{.r = 0x400, .n = 0x3, .m = 0xf, .expected = 0x40d, .mode = 0},
    Case{.r = 0x40f, .n = 0x3, .m = 0xf, .expected = 0x402, .mode = 1},
    Case{.r = 0x400, .n = 0x3, .m = 0xf, .expected = 0x40f, .mode = 2},
    Case{.r = 0x40f, .n = 0x3, .m = 0xf, .expected = 0x400, .mode = 3},
    // upper-modulo
    Case{.r = 0x400, .n = 0x3, .m = 0xa5000f, .expected = 0x40d, .mode = 0},
    Case{.r = 0x40f, .n = 0x3, .m = 0xa5000f, .expected = 0x402, .mode = 1},
    Case{.r = 0x400, .n = 0x3, .m = 0xa5000f, .expected = 0x40f, .mode = 2},
    Case{.r = 0x40f, .n = 0x3, .m = 0xa5000f, .expected = 0x400, .mode = 3},
    // multiple-wrap
    Case{.r = 0x400, .n = 0x13, .m = 0x8007, .expected = 0x405, .mode = 0},
    Case{.r = 0x407, .n = 0x13, .m = 0x8007, .expected = 0x402, .mode = 1},
    Case{.r = 0x400, .n = 0x13, .m = 0x8007, .expected = 0x407, .mode = 2},
    Case{.r = 0x407, .n = 0x13, .m = 0x8007, .expected = 0x400, .mode = 3},
    // reverse
    Case{.r = 0x0, .n = 0x800000, .m = 0x0, .expected = 0xffffff, .mode = 0},
    Case{.r = 0x800000, .n = 0x800000, .m = 0x0, .expected = 0x400000, .mode = 1},
    Case{.r = 0x0, .n = 0x800000, .m = 0x0, .expected = 0x1, .mode = 2},
    Case{.r = 0x800000, .n = 0x800000, .m = 0x0, .expected = 0x800001, .mode = 3},
};

inline unsigned opcode(const Case input, const unsigned reg) {
  return update_base | (input.mode << mode_shift) | (reg << register_shift);
}

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline DspDebugState
prepare(DspMachine &machine, const Case input, const unsigned reg, const bool window = false) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  constexpr unsigned accumulator_seed = 0x123456;
  program.immediate("a", accumulator_seed);
  program.immediate("m" + std::to_string(reg), input.m);
  program.immediate("r" + std::to_string(reg), input.r);
  program.immediate("n" + std::to_string(reg), input.n);
  program.immediate("sr", status_without_carry | (reg & 1U));
  for (unsigned index = 0; index < 8; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned index = 0; index < padding; ++index) {
    machine.bus().write({.address = entry + index, .value = 0});
  }
  machine.bus().write(
      {.address = entry + static_cast<unsigned>(window), .value = opcode(input, reg)});
  return machine.cpu().debug_state();
}

void native_checks();
void pipeline_checks();
void dependency_checks();
void guard_checks();
}
