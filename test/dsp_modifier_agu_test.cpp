// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100;
constexpr unsigned base = 0x400;
constexpr unsigned cookie = 0x123456;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned high_shift = 16;
// FM13-118: X0=00100,W=1,EA=1,R0; MMM=010/011 for post-decrement/increment.
constexpr unsigned load_decrement = 0x44d000;
constexpr unsigned load_increment = 0x44d800;
constexpr std::array upper_bytes{0U, 1U, 0x7fU, 0x80U, 0xffU};

bool one(const unsigned upper, const unsigned low, const bool decrement) {
  DspMachine machine;
  const auto modifier = (upper << high_shift) | low;
  const auto pointer = decrement ? base : base + low;
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("m0", modifier);
  program.immediate("r0", pointer);
  program.immediate("sr", masked_status);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  machine.bus().write({.address = entry, .value = decrement ? load_decrement : load_increment});
  machine.bus().write({.space = DspSpace::x, .address = pointer, .value = cookie});
  const auto before = machine.cpu().debug_state();
  require(before.cpu.m[0] == modifier && before.cpu.r[0] == pointer && before.cpu.pc == entry,
          "Own literal seeds retain the entire Mn data value and declared buffer endpoint");
  auto expected = before;
  // FM4-9 Table4-2: XX is arbitrary; low Mn plus one selects the declared modulo buffer size.
  expected.cpu.r[0] = decrement ? base + low : base;
  expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | cookie;
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  const auto saved = machine.checkpoint();
  machine.cpu().step();
  const auto actual = machine.cpu().debug_state();
  bool passed = actual == expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    DspMachine branch;
    branch.restore(saved);
    require(branch.cpu().debug_state() == before,
            "The native snapshot retains raw modifier and the full pre-access CPU state");
    branch.cpu().step();
    passed = passed && branch.cpu().debug_state() == expected;
  }
  std::cout << (passed ? "MATCH" : "MISMATCH") << " m=" << hex(modifier, 6)
            << " decrement=" << decrement << " actual_R0=" << hex(actual.cpu.r[0], 6)
            << " expected_R0=" << hex(expected.cpu.r[0], 6) << " raw_M0=" << hex(actual.cpu.m[0], 6)
            << '\n';
  return passed;
}
}

int main() {
  try {
    unsigned matches = 0;
    unsigned mismatches = 0;
    for (const unsigned low : {1U, 2U, 5U, 7U, 15U, 255U}) {
      for (const bool decrement : {false, true}) {
        for (const auto upper : upper_bytes) {
          if (one(upper, low, decrement)) {
            ++matches;
          } else {
            ++mismatches;
          }
        }
      }
    }
    std::cout << "Independent Mn high-byte snapshot audit:matches=" << matches
              << " mismatches=" << mismatches << '\n';
    return mismatches == 0 ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
