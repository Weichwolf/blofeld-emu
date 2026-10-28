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
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, first_cell = 0x3ff, last_cell = 0x408;
constexpr unsigned stored_value = 0x123456, source_value = 0xabcdef, sentinel = 0x654321;
constexpr unsigned plain_omr = 0x300, linear = 0xffffff;
constexpr unsigned x0_load = 0x44c000, x0_store = 0x444000, y_bit = 0x80000;
constexpr unsigned absolute_jump = 0x0af080;

struct Vector {
  unsigned mode, pointer, offset, modifier, address, after;
};

// FM4.4.2 and4.5: literal linear and eight-word circular address expectations.
constexpr std::array vectors{
    Vector{.mode = 5,
           .pointer = 0x403,
           .offset = 3,
           .modifier = linear,
           .address = 0x406,
           .after = 0x403},
    Vector{.mode = 5,
           .pointer = 0x403,
           .offset = 0xffffff,
           .modifier = linear,
           .address = 0x402,
           .after = 0x403},
    Vector{
        .mode = 5, .pointer = 0x407, .offset = 3, .modifier = 7, .address = 0x402, .after = 0x407},
    Vector{.mode = 5,
           .pointer = 0x400,
           .offset = 0xffffff,
           .modifier = 7,
           .address = 0x407,
           .after = 0x400},
    Vector{.mode = 7,
           .pointer = 0x400,
           .offset = 0x54321,
           .modifier = linear,
           .address = 0x3ff,
           .after = 0x3ff},
    Vector{.mode = 7,
           .pointer = 0x400,
           .offset = 0x765432,
           .modifier = 7,
           .address = 0x407,
           .after = 0x407}};

struct Case {
  Vector vector;
  unsigned reg, flags;
  bool y, load;
};

unsigned opcode(const Case sample) {
  return (sample.load ? x0_load : x0_store) | (sample.y ? y_bit : 0U) |
         (sample.vector.mode << 11U) | (sample.reg << 8U);
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  const auto reg = std::to_string(sample.reg);
  DspProgram program;
  program.immediate("omr", plain_omr);
  program.immediate("m" + reg, sample.vector.modifier);
  program.immediate("r" + reg, sample.vector.pointer);
  program.immediate("n" + reg, sample.vector.offset);
  program.immediate("x0", stored_value);
  program.immediate("x1", sentinel);
  program.immediate("y0", sentinel);
  program.immediate("y1", stored_value);
  program.immediate("sr", masked_status | sample.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first_cell; address <= last_cell; ++address) {
      const bool selected = (space == DspSpace::y) == sample.y && address == sample.vector.address;
      machine.bus().write({.space = space,
                           .address = address,
                           .value = selected && sample.load ? source_value : sentinel});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == plain_omr &&
              before.cpu.r.at(sample.reg) == sample.vector.pointer &&
              before.cpu.n.at(sample.reg) == sample.vector.offset &&
              before.cpu.m.at(sample.reg) == sample.vector.modifier &&
              (before.cpu.x & word_mask) == stored_value &&
              before.cpu.sr == (masked_status | sample.flags),
          "Real native setup establishes the exact independent address and flag inputs");
  return before;
}

void check(const Case sample) {
  DspMachine machine;
  auto expected = prepare(machine, sample);
  expected.cpu.r.at(sample.reg) = sample.vector.after;
  if (sample.load) {
    constexpr std::uint64_t high_half = 0xffffff000000;
    expected.cpu.x = (expected.cpu.x & high_half) | source_value;
  }
  ++expected.cpu.pc;
  ++expected.native_instructions;
  // FM A.1/TableA-1: effective-memory MOVE T=1 plus pre-update pru=1.
  expected.native_cycles += 2;
  machine.cpu().step();
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << "opcode=" << hex(opcode(sample), 6) << " R=" << hex(actual.cpu.r.at(sample.reg), 6)
              << '/' << hex(sample.vector.after, 6) << " X=" << hex(actual.cpu.x, 12) << '/'
              << hex(expected.cpu.x, 12) << " cycles=" << actual.native_cycles << '/'
              << expected.native_cycles << '\n';
  }
  require(actual == expected, "Independent pre-update full CPU/CCR/address/cycle expectation");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first_cell; address <= last_cell; ++address) {
      const bool selected = (space == DspSpace::y) == sample.y && address == sample.vector.address;
      const auto transferred = sample.load ? source_value : stored_value;
      const auto value = selected ? transferred : sentinel;
      require(machine.bus().read({.space = space, .address = address}) == value,
              "The real memory access retains neighboring and opposite-space sentinels");
    }
  }
}
}

int main() {
  try {
    unsigned count = 0;
    for (const auto vector : vectors) {
      for (const unsigned reg : {0U, 7U}) {
        for (const unsigned flags : {0U, 0x3fU, 0xc0U, 0xffU}) {
          for (const bool y : {false, true}) {
            for (const bool load : {false, true}) {
              check({.vector = vector, .reg = reg, .flags = flags, .y = y, .load = load});
              ++count;
            }
          }
        }
      }
    }
    std::cout << count << " independent native pre-update address/value/CCR/cycle cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
