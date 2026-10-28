// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
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

DspDebugState prepare(DspMachine &machine, const Case sample, const bool known_source) {
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
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first_cell; address <= last_cell; ++address) {
      const bool selected = (space == DspSpace::y) == sample.y && address == sample.vector.address;
      if (selected && sample.load && !known_source) { continue; }
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

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void unknown_source(const Case sample) {
  DspMachine machine;
  auto expected = prepare(machine, sample, false);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 6; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  expected.cpu.r.at(sample.reg) = sample.vector.after;
  require(machine.cpu().debug_state() == expected,
          "Unknown source retains real earlier AG2 through the extra E1 period");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "A delayed native read of unknown RAM must fail visibly");
    const std::string space = sample.y ? "Y:" : "X:";
    require(
        error.contains("read " + space + hex(sample.vector.address, 6)) &&
            error.contains("pc=0x000100") && error.contains("omr=0x000300") && run.faulted() &&
            run.completed_cycles() == 6 && machine.cpu().debug_state() == expected,
        "Delayed unknown E1 retains address/operation/PC/OMR and no invented data or retirement");
    const auto quarantined =
        checkpoint::rejects([&] { machine.cpu().step(); },
                            "A rejected native backend cannot continue through public step");
    require(quarantined.contains("requires reset after a rejected reference step") &&
                machine.cpu().debug_state() == expected,
            "A real public step confirms native quarantine without architectural mutation");
  }
  run.restore(saved, machine);
  machine.bus().write({.space = sample.y ? DspSpace::y : DspSpace::x,
                       .address = sample.vector.address,
                       .value = source_value});
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  constexpr std::uint64_t high_half = 0xffffff000000;
  expected.cpu.x = (expected.cpu.x & high_half) | source_value;
  ++expected.cpu.pc;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(machine.cpu().debug_state() == expected && !run.faulted() && run.completed_cycles() == 8,
          "Trusted restore plus independently supplied RAM permits the real native read and E2");
}
}

int main() {
  try {
    unsigned count = 0;
    for (const auto vector : vectors) {
      for (const unsigned reg : {0U, 7U}) {
        for (const unsigned flags : {0U, 0xffU}) {
          for (const bool y : {false, true}) {
            unknown_source({.vector = vector, .reg = reg, .flags = flags, .y = y, .load = true});
            ++count;
          }
        }
      }
    }
    std::cout << count
              << " delayed unknown-source cases,two failures/one repaired suffix each passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
