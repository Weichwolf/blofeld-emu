// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
enum class DspFetchPipeline : std::uint8_t { explicit_effects, automatic_alu, automatic_memory };

struct DspPipelineCycle {
  std::optional<DspProgramWord> decode = std::nullopt;
  std::optional<DspProgramWord> ag1 = std::nullopt;
  std::optional<DspProgramWord> ag2 = std::nullopt;
  std::optional<DspProgramWord> execute1 = std::nullopt;
  std::optional<DspProgramWord> execute2 = std::nullopt;
  std::optional<DspState> e1_operands = std::nullopt;
  std::optional<DspState> e2_operands = std::nullopt;
  // Functional early evaluation base; dependent A/B reads belong to E2 below.
  std::optional<DspState> candidate_inputs = std::nullopt;
  std::uint8_t e1_deferred_accumulators = 0;
  std::uint8_t e2_deferred_accumulators = 0;
  std::optional<DspReferenceProgress> execution = std::nullopt;
  std::optional<DspAddressStageView> ag1_address = std::nullopt;
  std::optional<DspAddressStageView> ag2_address = std::nullopt;
  std::optional<DspMemoryStageResult> memory = std::nullopt;
  std::optional<DspProgramWord> ag1_extension = std::nullopt;
  std::optional<DspProgramWord> e1_extension = std::nullopt;
  std::optional<DspMoveWriteback> transfer = std::nullopt;
  unsigned decode_words = 1;
  std::uint8_t agu_dependencies = 0;
  bool decode_accepted = false;
  bool memory_source_stall = false;
  bool retirement_published = false;
  bool execution_captured = false;
  bool advanced = false;
  // The actual extension word traverses Decode while its opcode occupies AG1.
  std::optional<DspProgramWord> extension_decode = std::nullopt;
  std::optional<DspProgramWord> extension_execute2 = std::nullopt;
};

struct DspPipelineView {
  // Decode/AG1/AG2 latches; E1 computes the actual native candidate for E2.
  std::array<std::optional<DspProgramWord>, 3> stages{};
  std::uint32_t decode_address = 0;
  std::optional<DspProgramWord> awaiting_retirement = std::nullopt;
  std::optional<DspState> awaiting_inputs = std::nullopt;
  std::uint8_t awaiting_deferred_accumulators = 0;
  DspPipelineCycle cycle{};
};
}
