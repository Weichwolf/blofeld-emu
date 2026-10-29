// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_accumulator_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <exception>
#include <iostream>
#include <string_view>

namespace blofeld::test::peripheral_accumulator {
namespace {
struct Oracle {
  DspDebugState expected;
  unsigned captures = 0, retired = 0;
};

void step(
    DspFetchRun &run, DspMachine &machine, const Input input, const bool flip, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto clock = cycle.completed_cycles;
  require(!phase.memory && !phase.memory_source_stall && !phase.ag2_address,
          "A fixed full-accumulator MOVEP must not invent a data writeback,AGU or source hold");
  if (cycle.bus.peripheral_read) {
    ++oracle.captures;
    require(clock == 4, "The actual full-word peripheral read occurs once in AG1");
    if (flip) { seed_source(machine, {.port = input.port, .value = input.value ^ 1U}); }
  }
  if (clock == 6) {
    load_result(oracle.expected, input);
    require(phase.transfer &&
                phase.transfer->fields == (input.b ? DspMoveFields{.b = 7} : DspMoveFields{.a = 7}),
            "E1 publishes exactly all three native accumulator fields");
  }
  const bool retires = clock >= 7;
  require(phase.retirement_published == retires,
          "The native MOVEP and six actual NOPs retire with independently expected one-period "
          "throughput");
  if (retires) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Each native retirement retains the real own program address");
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period agrees with independent full accumulator/CCR/CPU/stack/counter state");
}

void finish(
    DspFetchRun &run, DspMachine &machine, const Input input, const bool flip, Oracle &oracle) {
  constexpr unsigned end = 13;
  while (run.completed_cycles() < end) { step(run, machine, input, flip, oracle); }
  require(oracle.captures == 1 && oracle.retired == 7 &&
              run.elapsed_ticks() == end * pipeline::period,
          "One actual full-word sample and seven native retirements have exact nominal timing");
}

void window(const Input input, const unsigned cut = 0) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input)};
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  if (cut == 0) {
    finish(run, machine, input, false, oracle);
    return;
  }
  while (run.completed_cycles() < cut) { step(run, machine, input, true, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    oracle = prefix;
    finish(run, machine, input, true, oracle);
  }
}

void pipeline_checks() {
  matrix([](const Input input) {
    window(input);
    constexpr unsigned negative_limit = 0x800000;
    if (input.status != (masked_status | all_ccr) ||
        (input.value != negative_limit && input.value != word_mask)) {
      return;
    }
    for (unsigned cut = 3; cut <= 8; ++cut) { window(input, cut); }
  });
  std::cout << "768 per-period windows;96 changed-source cuts/192 independent suffixes passed\n";
}
}
}

int main(const int count, char **arguments) {
  try {
    using namespace blofeld::test::peripheral_accumulator;
    const std::string_view mode = count == 2 ? arguments[1] : "";
    if (mode == "native") {
      native_checks();
    } else if (mode == "pipeline") {
      pipeline_checks();
    } else if (mode == "dependencies") {
      dependency_checks();
    } else if (mode == "guards") {
      guard_checks();
    } else {
      throw blofeld::Fault("Expected native,pipeline,dependencies or guards");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
