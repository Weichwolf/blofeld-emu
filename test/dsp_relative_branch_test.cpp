// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned leading = 256;
constexpr unsigned padding = 512;
constexpr unsigned branch = boot + leading;
constexpr unsigned branch_cost = 4;
constexpr unsigned first_retirement = 7;
constexpr unsigned target_retirement = 11;

struct Case {
  unsigned opcode;
  unsigned target;
};

// Independent FM13-25 fields and unchanged-model cases,including signed limits.
constexpr std::array cases{Case{.opcode = 0x050c0d, .target = branch + 13},
                           Case{.opcode = 0x050c00, .target = branch},
                           Case{.opcode = 0x050c01, .target = branch + 1},
                           Case{.opcode = 0x050c02, .target = branch + 2},
                           Case{.opcode = 0x050c1f, .target = branch + 31},
                           Case{.opcode = 0x050c40, .target = branch + 32},
                           Case{.opcode = 0x050ddf, .target = branch + 255},
                           Case{.opcode = 0x050fdf, .target = branch - 1},
                           Case{.opcode = 0x050fc0, .target = branch - 32},
                           Case{.opcode = 0x050e00, .target = branch - 256}};

DspDebugState prepare(DspMachine &machine, const unsigned opcode, const unsigned prefix) {
  DspProgram code;
  for (unsigned index = 0; index < prefix + padding; ++index) {
    code.word(index == prefix ? opcode : 0U);
  }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  return machine.cpu().debug_state();
}

struct Expectation {
  unsigned pc;
  unsigned instructions;
  unsigned native_cycles;
};

void state(const DspMachine &machine, const DspDebugState &before, const Expectation result) {
  auto expected = before;
  expected.cpu.pc = result.pc;
  expected.native_instructions += result.instructions;
  expected.native_cycles += result.native_cycles;
  require(
      machine.cpu().debug_state() == expected,
      "Independent literal PC/costs and unchanged complete CPU/retained frames match native E2");
}

void one_case(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample.opcode, leading);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const bool self = sample.target == branch;
  const unsigned last = leading + (self ? first_retirement + (2 * branch_cost) : target_retirement);
  unsigned branches = 0;
  unsigned successors = 0;
  unsigned prefix_retired = 0;
  while (run.completed_cycles() < last) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if (phase.execute1 && phase.execute1->address == branch) {
      require(
          !phase.e1_extension && phase.execution_captured,
          "The native short branch consumes one actual F2 opcode without an invented extension");
    }
    if (!phase.retirement_published || !phase.execute2) { continue; }
    const auto pc = phase.execute2->address;
    if (prefix_retired < leading) {
      ++prefix_retired;
      state(machine,
            before,
            {.pc = boot + prefix_retired,
             .instructions = prefix_retired,
             .native_cycles = prefix_retired});
    } else if (pc == branch) {
      require(
          cycle.completed_cycles == leading + first_retirement + (branches * branch_cost),
          "Selected seven-stage E2 and steady branch spacing match the declared phase contract");
      ++branches;
      state(machine,
            before,
            {.pc = sample.target,
             .instructions = leading + branches,
             .native_cycles = leading + (branches * branch_cost)});
    } else {
      require(
          pc == sample.target && cycle.completed_cycles == leading + target_retirement,
          "The last-prefix-NOP to target-NOP window matches the independent four-plus-one cost");
      ++successors;
      state(machine,
            before,
            {.pc = sample.target + 1,
             .instructions = leading + 2,
             .native_cycles = leading + branch_cost + 1});
    }
  }
  require(
      prefix_retired == leading && branches == (self ? 3U : 1U) && successors == (self ? 0U : 1U) &&
          run.elapsed_ticks() == last * pipeline::period,
      "Every native branch and real target retires exactly once at its actual owned occurrence");
}

void restored_branch() {
  constexpr unsigned opcode = 0x050c0d;
  for (const unsigned cut : {4U, 5U, 6U, 7U}) {
    DspMachine machine;
    const auto before = prepare(machine, opcode, 0);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    auto contract = pipeline::contract();
    contract.pipeline = DspFetchPipeline::automatic_memory;
    DspFetchRun run(contract, machine);
    pipeline::clocks(run, machine, cut);
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(run.completed_cycles() == cut && !run.faulted(),
              "Joint restore retains the real single-word forecast/F2/native-retirement ownership");
      pipeline::clocks(run, machine, target_retirement - cut);
      state(
          machine, before, {.pc = boot + 14, .instructions = 2, .native_cycles = branch_cost + 1});
    }
  }
}
}

int main() {
  try {
    for (const auto sample : cases) { one_case(sample); }
    restored_branch();
    std::cout << "Ten actual native one-word branch paths and eight trusted continuations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
