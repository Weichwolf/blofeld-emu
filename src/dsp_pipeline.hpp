// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pipeline.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
void validate_alu_pipeline_word(DspProgramWord word, std::uint32_t omr);
[[nodiscard]] std::uint8_t deferred_alu_accumulators(DspProgramWord producer,
                                                     DspProgramWord consumer);

struct DspInstructionPipeline {
  DspFetchPipeline profile = DspFetchPipeline::automatic_alu;
  DspPipelineView view;
  std::array<std::optional<DspAddressStageTicket>, 2> addresses{};

  void prepare(const DspMachine &machine);
  void rebind(const DspMachine &machine);
  void capture_address(DspMachine &machine);
  [[nodiscard]] DspReferenceTicket retirement(const DspMachine &machine) const;
  void complete(DspMachine &machine, const DspReferenceCycleResult &bus);

private:
  void execute(DspMachine &machine);
};
}
