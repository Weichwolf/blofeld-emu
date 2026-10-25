// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pipeline.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <cstdint>

namespace blofeld {
void validate_alu_pipeline_word(DspProgramWord word, std::uint32_t omr);
[[nodiscard]] std::uint8_t deferred_alu_accumulators(DspProgramWord producer,
                                                     DspProgramWord consumer);

struct DspAluPipeline {
  DspPipelineView view;

  void prepare(const DspMachine &machine);
  [[nodiscard]] DspReferenceTicket retirement(const DspMachine &machine) const;
  void complete(DspMachine &machine, const DspReferenceCycleResult &bus);

private:
  void execute(DspMachine &machine);
};
}
