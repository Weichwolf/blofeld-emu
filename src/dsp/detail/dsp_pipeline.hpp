// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pipeline_words.hpp"
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
  bool branch = false;
};

[[nodiscard]] DspPipelineFormat
decode_pipeline_word(DspProgramWord word, std::uint32_t omr, DspFetchPipeline profile);
[[nodiscard]] unsigned pipeline_word_count(DspProgramWord word);
[[nodiscard]] bool pipeline_counted_loop(DspProgramWord word);
[[nodiscard]] bool pipeline_ccr_immediate(std::uint32_t opcode);
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
[[nodiscard]] bool pipeline_loop_source_dependency(DspInstructionDependency instructions);

struct DspPipelineLoopSetup {
  DspFetchedInstruction captured;
  std::optional<std::uint32_t> target;
  unsigned decode_hold = 0;
  bool target_ready = false;
  bool held_decode = false;
};

struct DspPipelineProgramMemory {
  DspFetchedWord captured;
  unsigned remaining = 5;
};

struct DspPipelineCcrSync {
  DspFetchedWord captured;
  unsigned decode_hold = 2;
  bool held_decode = false;
};

struct DspInstructionPipeline {
  DspFetchPipeline profile = DspFetchPipeline::automatic_alu;
  DspPipelineView view;
  std::array<std::optional<DspFetchedWord>, 3> samples{};
  std::array<std::optional<DspFetchedWord>, 2> extension_samples{};
  DspPipelineWords selected{};
  std::array<std::optional<DspAddressStageTicket>, 2> addresses{};
  std::array<std::optional<DspProgramWord>, 2> extensions{};
  std::array<std::optional<DspPeripheralReadTicket>, 2> reads{};
  std::optional<DspBranchDecision> branch = std::nullopt;
  unsigned branch_fetch_hold = 0;
  unsigned branch_decode_hold = 0;
  bool branch_target_ready = false;
  bool branch_target_decoded = false;
  bool branch_retired = false;
  bool held_branch_decode = false;
  std::optional<DspPipelineLoopSetup> loop_setup = std::nullopt;
  std::optional<DspPipelineProgramMemory> program_memory = std::nullopt;
  bool held_program_memory = false;
  std::optional<DspPipelineCcrSync> ccr_sync = std::nullopt;

  void prepare(const DspMachine &machine);
  void rebind(const DspMachine &machine);
  void capture_address(DspMachine &machine);

  [[nodiscard]] bool hold_fetch() const {
    return held_program_memory || branch_fetch_hold != 0 || held_branch_decode ||
           (loop_setup && (!loop_setup->target || loop_setup->held_decode));
  }

  [[nodiscard]] bool hold_loop_decode();
  void accept_loop_decode(DspMachine &machine);
  void capture_loop_target(DspMachine &machine);
  void capture_loop_f2(const DspReferenceCycleResult &bus);
  void prepare_program_memory(const DspMachine &machine, std::optional<DspProgramWord> word);
  void validate_program_decode(const DspMachine &machine, const DspPipelineCycle &cycle) const;
  void complete_program_memory();
  [[nodiscard]] bool hold_ccr_decode(const DspMachine &machine);
  void accept_ccr_decode();

  [[nodiscard]] std::uint64_t peripheral_occurrence() const;
  void finish_cycle();
  [[nodiscard]] DspReferenceTicket retirement(const DspMachine &machine) const;
  void complete(DspMachine &machine, const DspReferenceCycleResult &bus);

private:
  void execute(DspMachine &machine);
  void capture_branch(DspMachine &machine);
  void validate_branch(const DspInstructionRetire &retirement) const;
  void release_branch();
  void retire_branch();
  void accept_branch_decode();
};
}
