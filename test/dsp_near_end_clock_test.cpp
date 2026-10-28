// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_near_end_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <cstddef>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::near_end;

DspDebugState prepare(DspMachine &machine, const Program &code, const Case sample) {
  DspProgram image;
  image.word(move_omr);
  image.word(ordinary_omr);
  // FM5.4.3.3: initialize reset-undefined SZ through actual own instructions.
  image.word(move_sz);
  image.word(0);
  for (unsigned index = 0; index < seed_words; ++index) { image.word(code.words.at(index)); }
  image.install(machine.bus());
  for (std::size_t index = 0; index < code.words.size(); ++index) {
    machine.bus().write(
        {.address = ram + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  auto expected = initial(sample);
  expected.native_cycles = machine.cpu().debug_state().native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Actual ROM preamble matches independent full register/stack preconditions");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return expected;
}

void step(DspFetchRun &run, DspMachine &machine, Oracle &oracle, const Program &code) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto pc = oracle.expected.cpu.pc;
  const bool due = cycle.completed_cycles == oracle.next_e2;
  const auto where =
      "Near-end pc=" + hex(pc, 6) + " period=" + std::to_string(cycle.completed_cycles);
  require(phase.retirement_published == due, where + " exact independent E2 period");
  if (due) {
    require(phase.execute2 && phase.execute2->address == pc &&
                phase.execute2->value == code.instructions.at(pc - ram).opcode,
            where + " genuine expected instruction occurrence");
    retire(oracle, code);
  }
  require(machine.cpu().debug_state() == oracle.expected,
          where + " full CPU/counters/all16 stack cells");
  require(cycle.completed_cycles < maximum_periods, "Bounded near-end composition terminates");
}

unsigned one_case(const Case sample, const bool snapshots) {
  const auto code = program(sample);
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, code, sample), .loops = {}};
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const auto finish = [&] {
    while (oracle.expected.cpu.pc != code.stop) { step(run, machine, oracle, code); }
    require(oracle.loops.empty() && oracle.expected.cpu.sp == 0 && oracle.expected.sc == 0,
            "Original interpreter balances every nested loop/call frame");
  };
  if (!snapshots) {
    finish();
    return static_cast<unsigned>(run.completed_cycles());
  }
  unsigned cuts = 0;
  for (;;) {
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    const auto cut = run.completed_cycles();
    for (unsigned suffix = 0; suffix < 2; ++suffix) {
      run.restore(saved, machine);
      oracle = prefix;
      require(machine.cpu().debug_state() == oracle.expected && run.completed_cycles() == cut,
              "Joint restore retains independent full state and native continuation");
      finish();
    }
    ++cuts;
    run.restore(saved, machine);
    oracle = prefix;
    if (oracle.expected.cpu.pc == code.stop) { return cuts; }
    step(run, machine, oracle, code);
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select near-end streams or snapshots");
    const std::string_view mode(arguments[1]);
    require(mode == "streams" || mode == "snapshots", "Unknown near-end validation scope");
    const bool snapshots = mode == "snapshots";
    unsigned selected = 0;
    unsigned checked = 0;
    for (const auto sample : cases()) {
      if (snapshots && !snapshot_case(sample)) { continue; }
      checked += one_case(sample, snapshots);
      ++selected;
    }
    require(selected == (snapshots ? expected_snapshot_cases : expected_streams) &&
                checked == (snapshots ? expected_cuts : expected_periods),
            "Exact independent near-end reference catalogue");
    std::cout << mode << " passed " << selected << " cases," << checked
              << (snapshots ? " cuts/two suffixes\n" : " full-state periods\n");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
