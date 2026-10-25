// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::memory_pipeline {
inline constexpr unsigned seeds = 14;
inline constexpr unsigned body = fetch_clock::program + (2 * seeds);
inline constexpr unsigned first = 0x123456;
inline constexpr unsigned second = 0x654321;
inline constexpr unsigned data_base = 0x100;
inline constexpr unsigned linear_modifier = 0xffffff;
inline constexpr unsigned half = 0x400000;
inline constexpr unsigned shi_control = 0xffff91;
inline constexpr unsigned spi_configuration = 0xa9;
inline constexpr std::uint64_t product_plus_unit = 0x200001000000;

struct Setup {
  unsigned index = 0;
  unsigned pointer = 7;
  unsigned offset = 3;
  unsigned modifier = 7;
};

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline void prepare(DspMachine &machine,
                    const std::span<const std::string> instructions,
                    const Setup setup = {}) {
  std::vector<std::uint32_t> words;
  constexpr std::array initial{"move #>$123456,x0",
                               "move #>1,x1",
                               "move #>$400000,y0",
                               "move #>0,y1",
                               "move #>0,a0",
                               "move #>0,a1",
                               "move #>0,a2",
                               "move #>0,b0",
                               "move #>1,b1",
                               "move #>0,b2"};
  for (const auto *text : initial) { pipeline::append(words, text); }
  const auto reg = std::to_string(setup.index);
  for (const auto &text : {"move #>" + number(setup.pointer) + ",r" + reg,
                           "move #>" + number(setup.offset) + ",n" + reg,
                           "move #>" + number(setup.modifier) + ",m" + reg,
                           "move #>" + number(masked_status) + ",sr"}) {
    pipeline::append(words, text.c_str());
  }
  for (const auto &text : instructions) { pipeline::append(words, text.c_str()); }
  words.insert(words.end(), 32, 0);
  fetch_clock::prepare(machine, words);
  for (unsigned index = 0; index < seeds; ++index) {
    static_cast<void>(reference::finish(machine, {.pending = machine.reference_pending()}));
    static_cast<void>(machine.begin_reference_instruction(
        index + 1 == seeds ? pipeline::execution : reference::boundaries));
  }
  require(machine.cpu().state().pc == body, "Actual native seeds enter the own memory stream");
}

using pipeline::clocks;
using pipeline::stages;
void stream_checks();
void guard_checks();
}
