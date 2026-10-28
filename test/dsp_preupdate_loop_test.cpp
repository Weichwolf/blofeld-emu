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

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, guards = 4, setup = entry + guards, body = setup + 2;
constexpr unsigned loop_flag = 0x8000, data_base = 0x400, last_cell = 0x408;
constexpr unsigned stored = 0x234567, canary = 0x654321, do_immediate = 0x060080;
constexpr unsigned ordinary_omr = 0x300, absolute_jump = 0x0af080;
constexpr unsigned x0_load = 0x44c000, x0_store = 0x444000, y_bit = 0x80000;
constexpr unsigned bound = 128;

struct Case {
  unsigned mode = 5, reg = 0, count = 3, size = 1, position = 0;
  bool y = false, load = false;

  [[nodiscard]] unsigned move_pc() const { return body + position; }

  [[nodiscard]] unsigned after() const { return body + size; }

  [[nodiscard]] unsigned pointer() const { return mode == 5 ? data_base + 4 : data_base; }

  [[nodiscard]] unsigned opcode() const {
    return (load ? x0_load : x0_store) | (y ? y_bit : 0U) | (mode << 11U) | (reg << 8U);
  }

  [[nodiscard]] unsigned end_clock() const {
    // Existing DO Decode spacing: D=7, first D=13(single)/12(multi), E2=D+4.
    // Each executed pre-update move adds one explicit reference E1 hold.
    if (count == 0) { return 15; }
    const auto first_decode = size == 1 ? 13U : 12U;
    return first_decode + 4 + (count * size) - 1 + count;
  }
};

unsigned address(const Case input, const unsigned ordinal) {
  return input.mode == 5 ? data_base + 7 : data_base + 7 - ordinal;
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Case input) {
  DspProgram program;
  const auto reg = std::to_string(input.reg);
  program.immediate("omr", ordinary_omr);
  program.immediate("m" + reg, 7);
  program.immediate("r" + reg, input.pointer());
  program.immediate("n" + reg, 3);
  program.immediate("x0", stored);
  program.immediate("sr", masked_status | limit | sticky | carry | negative);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned index = 0; index < 32; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = setup, .value = do_immediate | (input.count << 8U)});
  machine.bus().write({.address = setup + 1, .value = input.after() - 1});
  machine.bus().write({.address = input.move_pc(), .value = input.opcode()});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned cell = data_base - 1; cell <= last_cell; ++cell) {
      const bool source = input.load && (space == DspSpace::y) == input.y;
      machine.bus().write({.space = space, .address = cell, .value = source ? cell : canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == ordinary_omr &&
              before.cpu.r.at(input.reg) == input.pointer() && before.cpu.n.at(input.reg) == 3 &&
              before.cpu.m.at(input.reg) == 7 && (before.cpu.x & word_mask) == stored,
          "Real native setup establishes independent DO and pre-update memory inputs");
  return before;
}

struct Oracle {
  DspDebugState expected;
  DspDebugState before;
  unsigned addresses = 0, transfers = 0, iterations = 0, holds = 0;
};

void open(Oracle &oracle, const Case input) {
  auto &wanted = oracle.expected;
  const auto before = oracle.before;
  wanted.native_cycles += 5; // FM A1 immediate DO cost, not the pipeline's elapsed counter.
  wanted.stack_high.at(1) = before.cpu.la;
  wanted.stack_low.at(1) = before.cpu.lc;
  wanted.stack_high.at(2) = body;
  wanted.stack_low.at(2) = before.cpu.sr;
  wanted.cpu.pc = input.count == 0 ? input.after() : body;
  if (input.count == 0) { return; }
  wanted.cpu.la = input.after() - 1;
  wanted.cpu.lc = input.count;
  wanted.cpu.sp = 2;
  wanted.sc = 2;
  wanted.cpu.sr |= loop_flag;
}

void retire(Oracle &oracle, const Case input) {
  auto &wanted = oracle.expected;
  const auto pc = wanted.cpu.pc;
  ++wanted.native_instructions;
  if (pc == setup) {
    open(oracle, input);
    return;
  }
  ++wanted.cpu.pc;
  wanted.native_cycles += pc == input.move_pc() ? 2U : 1U;
  if (pc != input.after() - 1) { return; }
  if (++oracle.iterations < input.count) {
    wanted.cpu.lc = input.count - oracle.iterations;
    wanted.cpu.pc = body;
    return;
  }
  wanted.cpu.la = oracle.before.cpu.la;
  wanted.cpu.lc = oracle.before.cpu.lc;
  wanted.cpu.sp = oracle.before.cpu.sp;
  wanted.sc = oracle.before.sc;
  wanted.cpu.sr = oracle.before.cpu.sr;
}

void cells(DspMachine &machine, const Case input, const unsigned writes) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned cell = data_base - 1; cell <= last_cell; ++cell) {
      auto wanted = input.load && (space == DspSpace::y) == input.y ? cell : canary;
      for (unsigned index = 0; !input.load && index < writes; ++index) {
        if (cell == address(input, index) && (space == DspSpace::y) == input.y) { wanted = stored; }
      }
      require(machine.bus().read({.space = space, .address = cell}) == wanted,
              "Independent per-iteration cells preserve neighbors and opposite X/Y memory");
    }
  }
}

void period(DspFetchRun &run, DspMachine &machine, const Case input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Counted memory loops lost actual pipeline evidence"); }
  const auto &phase = *cycle.pipeline;
  auto &wanted = oracle.expected;
  if (phase.ag2_address) {
    const auto &access = *phase.ag2_address;
    const auto cell = address(input, oracle.addresses++);
    require(access.word.address == input.move_pc() && access.update.address == cell &&
                access.pointer_update == (input.mode == 7) && access.published,
            "Each real loop occurrence publishes its independent AG2 address exactly once");
    if (input.mode == 7) { wanted.cpu.r.at(input.reg) = cell; }
  }
  if (phase.memory) {
    const auto cell = address(input, oracle.transfers++);
    require(phase.execute1 && phase.execute1->address == input.move_pc() &&
                phase.memory->data.target.address == cell && !phase.memory->additional_data,
            "Each delayed E1 accesses the correct independent iteration cell");
    if (input.load) { wanted.cpu.x = (wanted.cpu.x & ~std::uint64_t{word_mask}) | cell; }
  }
  if (!phase.advanced) {
    require(!phase.memory_source_stall && phase.execute1 &&
                phase.execute1->address == input.move_pc(),
            "Only the explicit one-period pre-update E1 hold delays these independent loops");
    ++oracle.holds;
  }
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == wanted.cpu.pc,
            "Native E2 follows independent prefix,DO,body,backedge and final fallthrough order");
    retire(oracle, input);
  }
  wanted.cpu.stack_pc = wanted.stack_high.at(wanted.cpu.sp);
  wanted.cpu.stack_sr = wanted.stack_low.at(wanted.cpu.sp);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "clock=" << cycle.completed_cycles << " mode=" << input.mode
              << " count=" << input.count << " size=" << input.size
              << " PC=" << hex(actual.cpu.pc, 6) << '/' << hex(wanted.cpu.pc, 6)
              << " cycles=" << actual.native_cycles << '/' << wanted.native_cycles << '\n';
  }
  require(actual == wanted, "Every period agrees with independent full CPU,CCR,stack and costs");
  cells(machine, input, oracle.transfers);
}

void finish(DspFetchRun &run, DspMachine &machine, const Case input, Oracle &oracle) {
  while (run.completed_cycles() < input.end_clock()) { period(run, machine, input, oracle); }
  const auto &wanted = oracle.expected;
  require(input.end_clock() < bound && wanted.cpu.pc == input.after() &&
              wanted.native_instructions == oracle.before.native_instructions + guards + 1 +
                                                (std::uint64_t{input.count} * input.size) &&
              oracle.addresses == input.count && oracle.transfers == input.count &&
              oracle.iterations == input.count && oracle.holds == input.count,
          "Complete native DO entry and exit matches independent elapsed and iteration totals");
}

void one(const Case input, const bool snapshots) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  Oracle oracle{.expected = before, .before = before};
  if (!snapshots) {
    finish(run, machine, input, oracle);
    return;
  }
  for (unsigned cut = 0; cut < input.end_clock(); ++cut) {
    const auto state = run.checkpoint(machine);
    const auto saved_oracle = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(state, machine);
      oracle = saved_oracle;
      finish(run, machine, input, oracle);
    }
    run.restore(state, machine);
    oracle = saved_oracle;
    period(run, machine, input, oracle);
  }
}

unsigned counts(Case input) {
  unsigned total = 0;
  for (const unsigned iterations : {0U, 1U, 3U, 4U}) {
    for (const unsigned size : {1U, 3U}) {
      for (unsigned position = 0; position < size; position += 2) {
        input.count = iterations;
        input.size = size;
        input.position = position;
        one(input, false);
        if (input.reg == 7 && iterations == 3) { one(input, true); }
        ++total;
      }
    }
  }
  return total;
}

unsigned matrix() {
  unsigned total = 0;
  for (const unsigned mode : {5U, 7U}) {
    for (const bool y : {false, true}) {
      for (const bool load : {false, true}) {
        for (const unsigned reg : {0U, 7U}) {
          total += counts({.mode = mode, .reg = reg, .y = y, .load = load});
        }
      }
    }
  }
  return total;
}
}

int main() {
  try {
    std::cout << matrix() << " genuine counted DO/pre-update memory cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
