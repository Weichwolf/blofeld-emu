// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_only.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp_address_update_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <exception>
#include <iostream>
#include <string_view>

namespace blofeld::test::address_update {
namespace {
struct Oracle {
  DspDebugState expected;
  unsigned effects = 0, retired = 0;
};

void step(
    DspFetchRun &run, DspMachine &machine, const Case input, const unsigned reg, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = cycle.completed_cycles;
  require(!phase.memory_source_stall && phase.agu_dependencies == 0 && !phase.memory,
          "An implicit U update has no explicit register destination or data operand");
  for (unsigned index = 0; index < cycle.bus.core_access_count; ++index) {
    require(cycle.bus.core_accesses.at(index).target.space == DspSpace::p,
            "The U window owns only actual instruction fetches and never a fake data footprint");
  }
  if (period == 5 || period == 6) {
    const auto address = period == 5 ? phase.ag1_address : phase.ag2_address;
    require(address && address->kind == DspAddressStageKind::address_only &&
                address->data_access_count == 0 && !address->extension && !address->secondary &&
                address->pointer_update && address->published == (period == 6) &&
                address->update.index == reg && address->update.before == input.r &&
                address->update.value == input.expected,
            "Actual F2/AG1 ownership captures exactly one independently checked pointer update");
    if (period == 6) { oracle.expected.cpu.r.at(reg) = input.expected; }
  }
  if (phase.address_only) {
    ++oracle.effects;
    require(period == 7 && phase.address_only->address.word.address == entry + 1 &&
                phase.address_only->progress.pending && phase.execution_captured,
            "E1 confirms the genuine native result from the captured pre-update source");
  }
  const bool retires = period >= 7 && period <= 13;
  require(phase.retirement_published == retires,
          "FM A-1 U and surrounding NOPs each retire at nominal one-period intervals");
  if (retires) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Only the actual sequential own instruction retires");
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period matches complete independent CPU/stack/flags/native-counter expectations");
}

void finish(
    DspFetchRun &run, DspMachine &machine, const Case input, const unsigned reg, Oracle &oracle) {
  while (run.completed_cycles() < 13) { step(run, machine, input, reg, oracle); }
  require(oracle.retired == 7 && oracle.effects == 1 &&
              run.elapsed_ticks() == 13 * pipeline::period,
          "The entire U window retains nominal clocks and exactly one native result");
}

struct Window {
  unsigned reg = 0, cut = 0;
};

void window(const Case input, const Window sample) {
  const auto reg = sample.reg;
  const auto cut = sample.cut;
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input, reg, true)};
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  if (cut == 0) {
    finish(run, machine, input, reg, oracle);
    return;
  }
  while (run.completed_cycles() < cut) { step(run, machine, input, reg, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    oracle = prefix;
    finish(run, machine, input, reg, oracle);
  }
}
}

void pipeline_checks() {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (const auto input : cases) {
      const auto form = decode_address_only(opcode(input, reg));
      require(form && form->reg == reg && form->mode == input.mode,
              "All32 independent vendor-pinned U encodings decode the documented operands");
      window(input, {.reg = reg});
      if (input.m == word_mask || input.m == 15) {
        for (unsigned cut = 4; cut <= 9; ++cut) { window(input, {.reg = reg, .cut = cut}); }
      }
    }
  }
  require(!decode_address_only(word_mask + 1) && !decode_address_only(0),
          "Out-of-word and non-U encodings do not acquire address continuations");
  std::cout << "192 full-state U pipeline windows and directed two-suffix phase cuts passed\n";
}
}

int main(const int count, char **arguments) {
  try {
    using namespace blofeld::test::address_update;
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
