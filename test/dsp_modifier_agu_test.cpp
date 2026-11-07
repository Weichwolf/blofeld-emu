// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <tuple>

namespace {
using namespace blofeld;
using test::esai::require;
namespace native = test::dma_test;
constexpr unsigned entry = 0x100;
constexpr unsigned base = 0x400;
constexpr unsigned cookie = 0x123456;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned high_shift = 16;
constexpr unsigned word_mask = 0xffffff, masked_status = 0xc00300;
// FM13-131: immediate MOVEC to OMR/M0/SR; FM13-118: immediate MOVE to R0.
constexpr unsigned move_omr = 0x05f43a, move_m0 = 0x05f420;
constexpr unsigned move_sr = 0x05f439, move_r0 = 0x60f400;
constexpr unsigned setup_limit = 32, setup_padding = 5;
// FM13-118: X0=00100,W=1,EA=1,R0; MMM=010/011 for post-decrement/increment.
constexpr unsigned load_decrement = 0x44d000;
constexpr unsigned load_increment = 0x44d800;
constexpr std::array upper_bytes{0U, 1U, 0x7fU, 0x80U, 0xffU};

auto observe(const DspMachine &machine) {
  return std::tuple{machine.cpu().registers(),
                    machine.cpu().progress(),
                    machine.cpu().fault(),
                    machine.cpu().processing_state(),
                    machine.clock().elapsed_ticks()};
}

void prepare(DspMachine &machine, const unsigned modifier, const unsigned pointer) {
  native::NativeProgram program;
  for (const auto word :
       {move_omr, ordinary_omr, move_m0, modifier, move_r0, pointer, move_sr, masked_status}) {
    program.word(word);
  }
  for (unsigned index = 0; index < setup_padding; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  native::reset_native(machine, program);
  for (unsigned count = 0; machine.cpu().registers().pc != entry && count < setup_limit; ++count) {
    native::step_native(machine);
  }
  require(machine.cpu().registers().pc == entry, "Literal setup reaches its actual jump target");
}

bool one(const unsigned upper, const unsigned low, const bool decrement) {
  DspMachine machine(native::native_clocks);
  const auto modifier = (upper << high_shift) | low;
  const auto pointer = decrement ? base : base + low;
  prepare(machine, modifier, pointer);
  machine.bus().write({.address = entry, .value = decrement ? load_decrement : load_increment});
  machine.bus().write({.space = DspSpace::x, .address = pointer, .value = cookie});
  const auto before = observe(machine);
  const auto &registers = std::get<0>(before);
  require(registers.m[0] == modifier && registers.r[0] == pointer && registers.pc == entry,
          "Own literal seeds retain the entire Mn data value and declared buffer endpoint");
  auto expected = before;
  // FM4-9 Table4-2: XX is arbitrary; low Mn plus one selects the declared modulo buffer size.
  auto &expected_registers = std::get<0>(expected);
  auto &expected_progress = std::get<1>(expected);
  expected_registers.r[0] = decrement ? base + low : base;
  expected_registers.x = (expected_registers.x & ~std::uint64_t{word_mask}) | cookie;
  ++expected_registers.pc;
  ++expected_progress.instructions;
  ++expected_progress.estimated_cycles;
  const auto saved = machine.checkpoint();
  native::step_native(machine);
  const auto actual = observe(machine);
  bool passed = actual == expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    DspMachine branch(native::native_clocks);
    branch.restore(saved);
    require(observe(branch) == before,
            "The native snapshot retains raw modifier and the full pre-access CPU state");
    native::step_native(branch);
    passed = passed && observe(branch) == expected;
  }
  std::cout << (passed ? "MATCH" : "MISMATCH") << " m=" << hex(modifier, 6)
            << " decrement=" << decrement << " actual_R0=" << hex(std::get<0>(actual).r[0], 6)
            << " expected_R0=" << hex(expected_registers.r[0], 6)
            << " raw_M0=" << hex(std::get<0>(actual).m[0], 6) << '\n';
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
