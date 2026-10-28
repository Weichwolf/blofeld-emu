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

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, first_cell = 0x3ff, last_cell = 0x408;
constexpr unsigned stored_value = 0x123456, source_value = 0xabcdef, sentinel = 0x654321;
constexpr unsigned plain_omr = 0x300;
constexpr unsigned x0_load = 0x44c000, x0_store = 0x444000, y_bit = 0x80000;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned full_flags = 0xff;

struct Vector {
  unsigned mode, pointer, offset, modifier, address, after;
};

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

struct Window {
  unsigned first_mode, reg, flags, gap;
  bool source_y, target_y;
};

constexpr unsigned initial_pointer = 0x402, indexed_source = 0x403, predecrement_source = 0x401;
constexpr unsigned indexed_target = 0x401, predecrement_target = 0x400, predecrement_mode = 7;

Case first_case(const Window window) {
  const bool indexed = window.first_mode == 5;
  return {.vector = {.mode = window.first_mode,
                     .pointer = initial_pointer,
                     .offset = 1,
                     .modifier = 7,
                     .address = indexed ? indexed_source : predecrement_source,
                     .after = indexed ? initial_pointer : predecrement_source},
          .reg = window.reg,
          .flags = window.flags,
          .y = window.source_y,
          .load = true};
}

unsigned target(const Window window) {
  return window.first_mode == 5 ? indexed_target : predecrement_target;
}

unsigned end_cycle(const Window window) {
  return 10 + window.gap;
}

DspDebugState prepare_window(DspMachine &machine, const Window window) {
  const auto before = prepare(machine, first_case(window));
  const auto second =
      x0_store | (window.target_y ? y_bit : 0U) | (predecrement_mode << 11U) | (window.reg << 8U);
  machine.bus().write({.address = entry + 1 + window.gap, .value = second});
  return before;
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState expected_state(DspDebugState before, const Window window, const std::uint64_t count) {
  const auto first = first_case(window);
  if (count >= 5) { before.cpu.r.at(window.reg) = first.vector.after; }
  if (count >= 7 + window.gap) { before.cpu.r.at(window.reg) = target(window); }
  if (count >= 7) {
    constexpr std::uint64_t high_half = 0xffffff000000;
    before.cpu.x = (before.cpu.x & high_half) | source_value;
  }
  if (count >= 8) {
    ++before.cpu.pc;
    ++before.native_instructions;
    before.native_cycles += 2;
  }
  if (window.gap != 0 && count >= 9) {
    ++before.cpu.pc;
    ++before.native_instructions;
    ++before.native_cycles;
  }
  if (count >= end_cycle(window)) {
    ++before.cpu.pc;
    ++before.native_instructions;
    before.native_cycles += 2;
  }
  return before;
}

void window_cells(DspMachine &machine, const Window window, const std::uint64_t count) {
  const auto first = first_case(window);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first_cell; address <= last_cell; ++address) {
      auto value = sentinel;
      if ((space == DspSpace::y) == window.source_y && address == first.vector.address) {
        value = source_value;
      }
      if ((space == DspSpace::y) == window.target_y && address == target(window) &&
          count >= 9 + window.gap) {
        value = source_value;
      }
      require(machine.bus().read({.space = space, .address = address}) == value,
              "Each actual window period preserves source and publishes only the selected store");
    }
  }
}

void window_period(DspFetchRun &run,
                   DspMachine &machine,
                   const DspDebugState &before,
                   const Window window) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto count = cycle.completed_cycles;
  const auto expected = expected_state(before, window, count);
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << "cycle=" << count << " first-mode=" << window.first_mode << " gap=" << window.gap
              << " R=" << hex(actual.cpu.r.at(window.reg), 6) << '/'
              << hex(expected.cpu.r.at(window.reg), 6) << " PC=" << hex(actual.cpu.pc, 6) << '/'
              << hex(expected.cpu.pc, 6) << " native=" << actual.native_instructions << '/'
              << expected.native_instructions << '\n';
  }
  require(actual == expected,
          "Independent older-E2/younger-AG2 full CPU/CCR and nominal-cycle window");
  if (!cycle.pipeline) { throw Fault("A pre-update window lost its actual stage record"); }
  const auto &phase = *cycle.pipeline;
  const bool held = count == 6 || count == 8 + window.gap;
  const bool memory = count == 7 || count == 9 + window.gap;
  require(!phase.memory_source_stall && phase.advanced == !held &&
              phase.memory.has_value() == memory,
          "Two real pre-update instructions have exactly two separate E1 holds and data accesses");
  if (phase.memory) {
    const auto &access = phase.memory->data;
    const bool first = count == 7;
    const auto kind = first ? DspNativeAccessKind::read : DspNativeAccessKind::write;
    const bool y = first ? window.source_y : window.target_y;
    const auto space = y ? DspSpace::y : DspSpace::x;
    require(access.kind == kind && access.target.space == space && access.value == source_value &&
                access.target.address ==
                    (first ? first_case(window).vector.address : target(window)),
            "Native receipt ordering retains actual loaded data as the younger store source");
  }
  window_cells(machine, window, count);
}

void one(const Window window) {
  DspMachine machine;
  const auto before = prepare_window(machine, window);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < end_cycle(window); ++index) {
    window_period(run, machine, before, window);
  }
}

void snapshots(const Window window) {
  for (unsigned cut = 0; cut < end_cycle(window); ++cut) {
    DspMachine machine;
    const auto before = prepare_window(machine, window);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned index = 0; index < cut; ++index) { window_period(run, machine, before, window); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned index = cut; index < end_cycle(window); ++index) {
        window_period(run, machine, before, window);
      }
    }
  }
}
}

int main() {
  try {
    unsigned count = 0;
    for (const unsigned mode : {5U, 7U}) {
      for (const unsigned gap : {0U, 1U}) {
        for (const unsigned reg : {0U, 7U}) {
          for (const unsigned flags : {0U, 0x3fU, 0xc0U, 0xffU}) {
            for (const bool source_y : {false, true}) {
              for (const bool target_y : {false, true}) {
                one({.first_mode = mode,
                     .reg = reg,
                     .flags = flags,
                     .gap = gap,
                     .source_y = source_y,
                     .target_y = target_y});
                ++count;
              }
            }
          }
        }
        snapshots({.first_mode = mode,
                   .reg = 7,
                   .flags = full_flags,
                   .gap = gap,
                   .source_y = false,
                   .target_y = true});
      }
    }
    std::cout << count << " pre-update load/store windows,42 cuts/84 suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
