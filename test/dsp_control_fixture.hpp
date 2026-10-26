// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <cstdint>
#include <span>

namespace blofeld::test::control {
inline constexpr unsigned sr_move = 0x05f439;
inline constexpr unsigned and_ccr = 0x0000b9;
inline constexpr unsigned indirect_jump = 0x0ae080;
inline constexpr unsigned padding = 64;
inline constexpr unsigned first_retirement = 7;

inline DspDebugState prepare(DspMachine &machine,
                             const std::span<const std::uint32_t> seeds,
                             const unsigned seed_instructions,
                             const std::span<const std::uint32_t> body) {
  DspProgram code;
  for (const auto word : seeds) { code.word(word); }
  for (const auto word : body) { code.word(word); }
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == boot + seeds.size(), "Actual seed instructions reach the own body");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline void check(const DspMachine &machine, const DspDebugState &expected) {
  require(machine.cpu().debug_state() == expected,
          "Complete native CPU, retained frames and instruction costs match literal expectations "
          "pc=" +
              hex(machine.cpu().state().pc, 6) + " expected=" + hex(expected.cpu.pc, 6));
}
}
