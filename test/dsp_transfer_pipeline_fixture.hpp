// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <span>
#include <string>
#include <vector>

namespace blofeld::test::transfer_pipeline {
using memory_pipeline::clocks;
using memory_pipeline::contract;
using memory_pipeline::stages;
inline constexpr unsigned body = memory_pipeline::body + 8;
inline constexpr unsigned sample = 0x800001;
inline constexpr unsigned other = 0x234567;
inline constexpr unsigned replacement = 0x654321;

struct Setup {
  unsigned index = 0;
  unsigned source = 1;
};

inline void
prepare(DspMachine &machine, const std::span<const std::string> code, const Setup setup = {}) {
  const auto secondary = (setup.index + 4) % 8;
  const auto reg = std::to_string(secondary);
  std::vector<std::string> program{"move #>7,r" + reg,
                                   "move #>3,n" + reg,
                                   "move #>7,m" + reg,
                                   "move #>" + number(setup.source) + ",x1"};
  program.insert(program.end(), code.begin(), code.end());
  memory_pipeline::prepare(machine, program, {.index = setup.index});
  for (unsigned index = 0; index < 4; ++index) {
    static_cast<void>(reference::finish(machine, {.pending = machine.reference_pending()}));
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  }
  require(machine.cpu().state().pc == body, "Actual native setup enters own transfer stream");
}

void field_checks();
void dependency_checks();
void state_checks();
}
