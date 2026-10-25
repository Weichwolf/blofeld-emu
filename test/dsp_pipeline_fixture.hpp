// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pipeline.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::pipeline {
inline constexpr unsigned seed_steps = 11;
inline constexpr unsigned body = fetch_clock::program + (2 * seed_steps);
inline constexpr std::uint64_t unit = std::uint64_t{1} << word_bits;
inline constexpr std::uint64_t period = 45;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

inline DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = execution;
  result.pipeline = DspFetchPipeline::automatic_alu;
  return result;
}

inline void append(std::vector<std::uint32_t> &program, const char *text) {
  const auto words = checkpoint::assemble(text);
  program.insert(program.end(), words.begin(), words.end());
}

struct Setup {
  bool dma = false;
  bool timer = false;
  unsigned status = masked_status | carry | negative;
  unsigned padding = 32;
  unsigned pointer = fetch_clock::initial_r0;
};

inline void prepare(DspMachine &machine,
                    const std::span<const char *const> instructions,
                    const Setup setup = {}) {
  std::vector<std::uint32_t> program;
  constexpr std::array seeds{"move #>$400000,x0",
                             "move #>1,x1",
                             "move #>$400000,y0",
                             "move #>0,a0",
                             "move #>0,a1",
                             "move #>0,a2",
                             "move #>0,b0",
                             "move #>1,b1",
                             "move #>0,b2"};
  for (const auto *const text : seeds) { append(program, text); }
  append(program, ("move #>" + number(setup.pointer) + ",r0").c_str());
  append(program, ("move #>" + number(setup.status) + ",sr").c_str());
  for (const auto *const text : instructions) { append(program, text); }
  program.insert(program.end(), setup.padding, 0);
  fetch_clock::prepare(machine,
                       program,
                       {.transfer = {.source = fetch_clock::program_source,
                                     .destination = body,
                                     .control = setup.dma ? fetch_clock::program_copy : 0},
                        .timer = setup.timer,
                        .execution = reference::boundaries});
  for (unsigned index = 0; index < seed_steps; ++index) {
    static_cast<void>(reference::finish(machine, {.pending = machine.reference_pending()}));
    const auto next = index + 1 == seed_steps ? execution : reference::boundaries;
    static_cast<void>(machine.begin_reference_instruction(next));
  }
  require(machine.cpu().state().pc == body, "Actual native setup enters the own ALU stream");
}

inline void clocks(DspFetchRun &run, DspMachine &machine, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
}

inline const DspPipelineCycle &stages(const DspFetchCycleResult &cycle) {
  if (!cycle.pipeline) { throw Fault("Expected an actual automatically scheduled cycle"); }
  return *cycle.pipeline;
}

void state_checks();
void guard_checks();
}
