// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
struct DspPipelineFormat {
  unsigned words = 1;
  bool memory = false;
  bool transfer = false;
};

[[nodiscard]] DspPipelineFormat
decode_pipeline_word(DspProgramWord word, std::uint32_t omr, DspFetchPipeline profile);
[[nodiscard]] unsigned pipeline_word_count(DspProgramWord word);
[[nodiscard]] DspMoveFields pipeline_transfer_fields(std::uint32_t opcode);
[[nodiscard]] std::uint8_t
pipeline_agu_dependencies(const std::array<std::optional<DspProgramWord>, 3> &preceding,
                          DspProgramWord consumer);
[[nodiscard]] std::optional<DspProgramWord> pipeline_extension(const DspProgramFetchView &fetch,
                                                               DspProgramWord instruction);
[[nodiscard]] std::uint8_t deferred_alu_accumulators(DspProgramWord producer,
                                                     DspProgramWord consumer);

struct DspInstructionDependency {
  DspProgramWord producer;
  DspProgramWord consumer;
};

[[nodiscard]] std::uint8_t memory_source_dependency(DspInstructionDependency instructions);

struct DspInstructionPipeline {
  DspFetchPipeline profile = DspFetchPipeline::automatic_alu;
  DspPipelineView view;
  std::array<std::optional<DspAddressStageTicket>, 2> addresses{};
  std::array<std::optional<DspProgramWord>, 2> extensions{};

  void prepare(const DspMachine &machine);
  void rebind(const DspMachine &machine);
  void capture_address(DspMachine &machine);
  [[nodiscard]] DspReferenceTicket retirement(const DspMachine &machine) const;
  void complete(DspMachine &machine, const DspReferenceCycleResult &bus);

private:
  void execute(DspMachine &machine);
};
}
