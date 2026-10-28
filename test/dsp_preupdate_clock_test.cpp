// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
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
constexpr unsigned full_flags = 0xff;

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
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
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

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void cells(DspMachine &machine, const Case sample, const std::uint64_t count) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first_cell; address <= last_cell; ++address) {
      const bool selected = (space == DspSpace::y) == sample.y && address == sample.vector.address;
      auto value = sentinel;
      if (selected && sample.load) { value = source_value; }
      if (selected && !sample.load && count >= 7) { value = stored_value; }
      require(machine.bus().read({.space = space, .address = address}) == value,
              "Every reference period preserves neighboring and opposite-space canaries");
    }
  }
}

void period(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto count = cycle.completed_cycles;
  auto expected = before;
  if (count >= 5) { expected.cpu.r.at(sample.reg) = sample.vector.after; }
  if (count >= 7 && sample.load) {
    constexpr std::uint64_t high_half = 0xffffff000000;
    expected.cpu.x = (expected.cpu.x & high_half) | source_value;
  }
  if (count == 8) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    expected.native_cycles += 2;
  }
  require(machine.cpu().debug_state() == expected,
          "Explicit pre-update reference retains full CPU/CCR through AG2,held E1,data and E2");
  if (!cycle.pipeline) { throw Fault("An automatic pre-update cycle lost its stage record"); }
  const auto &phase = *cycle.pipeline;
  require(!phase.memory_source_stall && phase.memory.has_value() == (count == 7) &&
              phase.advanced == (count != 6),
          "One owned E1 hold delays exactly one real data access without a source dependency");
  if (count == 4) {
    const auto &address = phase.ag1_address;
    require(address && address->word.value == opcode(sample) && !address->published &&
                address->pointer_update == (sample.vector.mode == 7) &&
                address->update.before == sample.vector.pointer &&
                address->update.value == sample.vector.after &&
                address->update.address == sample.vector.address,
            "Actual native AG1 retains the independent address and conditional pointer update");
  }
  if (count == 7) {
    if (!phase.memory) { throw Fault("Delayed pre-update E1 lost its actual memory receipt"); }
    const auto &memory = *phase.memory;
    const auto kind = sample.load ? DspNativeAccessKind::read : DspNativeAccessKind::write;
    require(
        memory.data.kind == kind && memory.data.pc == entry && memory.data.omr == plain_omr &&
            memory.data.target.address == sample.vector.address &&
            memory.data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
            memory.data.value == (sample.load ? source_value : stored_value),
        "Delayed E1 retains the exact real native read/write,source value and execution context");
  }
  cells(machine, sample, count);
}

void automatic(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 8; ++cycle) { period(run, machine, before, sample); }
}

void snapshots(const Case sample) {
  for (unsigned cut = 0; cut < 8; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, sample);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned index = 0; index < cut; ++index) { period(run, machine, before, sample); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned index = cut; index < 8; ++index) { period(run, machine, before, sample); }
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
              automatic({.vector = vector, .reg = reg, .flags = flags, .y = y, .load = load});
              ++count;
            }
          }
        }
      }
      for (const bool load : {false, true}) {
        snapshots({.vector = vector, .reg = 7, .flags = full_flags, .y = true, .load = load});
      }
    }
    std::cout << count << " pre-update reference cases,96 cuts/192 suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
