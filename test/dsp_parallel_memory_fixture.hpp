// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <span>
#include <string>
#include <vector>

namespace blofeld::test::parallel_memory {
inline constexpr unsigned body = memory_pipeline::body + 6;
inline constexpr unsigned initial = 7;
inline constexpr unsigned offset = 3;
inline constexpr unsigned modifier = 7;
inline constexpr unsigned word_x = 0x234567;
inline constexpr unsigned word_y = 0x345678;
using memory_pipeline::clocks;
using memory_pipeline::contract;
using memory_pipeline::stages;

struct Setup {
  memory_pipeline::Setup primary;
  unsigned secondary = 4;
  unsigned secondary_pointer = initial;
};

inline void
prepare(DspMachine &machine, const std::span<const std::string> code, const Setup setup = {}) {
  require(setup.primary.index != setup.secondary,
          "Own dual address seeds use distinct native registers");
  const auto reg = std::to_string(setup.secondary);
  std::vector<std::string> program{"move #>" + number(setup.secondary_pointer) + ",r" + reg,
                                   "move #>3,n" + reg,
                                   "move #>7,m" + reg};
  program.insert(program.end(), code.begin(), code.end());
  memory_pipeline::prepare(machine, program, setup.primary);
  for (unsigned index = 0; index < 3; ++index) {
    static_cast<void>(reference::finish(machine, {.pending = machine.reference_pending()}));
    static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  }
  require(machine.cpu().state().pc == body, "Actual native dual-address setup enters own code");
}

void address_checks();
void execution_checks();
void state_checks();
}
