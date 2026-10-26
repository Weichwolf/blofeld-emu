// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
// Occurrence metadata, not permission to perform native or peripheral effects.
struct DspFetchedWord {
  DspProgramWord word;
  std::uint64_t occurrence = 0;
  bool operator==(const DspFetchedWord &) const = default;
};

struct DspInstructionRetire;

struct DspFetchOrigin {
  std::uint32_t omr = 0;
  DspMemoryResource resource;
  bool operator==(const DspFetchOrigin &) const = default;
};

class DspProgramFetch;

struct DspFetchedInstruction {
  DspFetchedWord opcode;
  std::optional<DspFetchedWord> extension;
  void check_front(const DspProgramFetch &fetch) const;
  void check_consumed(const std::array<DspFetchedWord, 2> &actual, std::uint32_t omr) const;
  bool operator==(const DspFetchedInstruction &) const = default;
};

struct DspFetchContext {
  DspFetchedInstruction writer;
  std::uint32_t before = 0;
  std::uint32_t after = 0;
};

struct DspFetchCommit {
  std::uint32_t pc;
  std::uint32_t next_pc;
  std::uint32_t omr;
  bool redirected;
};

// Owner-driven actual F1/F2; native CPU execution still owns architectural loops.
class DspProgramFetch {
public:
  [[nodiscard]] DspProgramFetchView view() const { return state_; }

  [[nodiscard]] bool active() const { return state_.active; }

  [[nodiscard]] bool pending_context() const { return context_.has_value(); }

  [[nodiscard]] const std::optional<DspFetchContext> &context() const { return context_; }

  [[nodiscard]] DspFetchOrigin origin(DspFetchedWord captured) const;
  [[nodiscard]] std::uint32_t execution_omr(DspFetchedWord captured) const;
  void stage_context(const DspInstructionRetire &result, const DspBus &bus);
  [[nodiscard]] bool confirm_context(DspFetchCommit completed, const DspBus &bus);

  void start(std::uint32_t pc, std::uint32_t omr);
  void check_stage(DspProgramFetchStage stage) const;
  void issue();
  void capture(DspBus &bus);
  [[nodiscard]] std::optional<std::uint32_t> peek(std::uint32_t address) const;
  [[nodiscard]] std::optional<DspFetchedWord> queued(std::size_t index) const;
  [[nodiscard]] DspFetchedWord select(std::uint32_t address, std::uint64_t identity = 0) const;
  [[nodiscard]] DspFetchedInstruction instruction(DspFetchedWord opcode, unsigned words) const;
  // Stop speculative fetch after the actual decoded two-word instruction.
  // This neither opens a native loop nor authorizes architectural effects.
  void hold_after(const DspFetchedInstruction &captured);
  [[nodiscard]] bool owns_loop_result(DspLoopArchitecture native_result, bool redirected) const;
  // Occurrence-bound forecast metadata, never an architectural CPU result.
  [[nodiscard]] std::optional<std::uint32_t> loop_last(DspFetchedWord captured) const;
  [[nodiscard]] std::uint32_t continuation_after(const DspFetchedInstruction &captured) const;
  [[nodiscard]] std::optional<DspProgramWord> occurrence(std::uint64_t identity) const;

  [[nodiscard]] std::array<DspFetchedWord, 2> consumed_pair() const { return consumed_; }

  [[nodiscard]] bool accepts_loop_control(DspLoopArchitecture native_result) const;
  void check_consume(std::uint32_t address) const;
  [[nodiscard]] std::uint32_t consume(std::uint32_t address);
  void redirect(std::uint32_t pc, std::uint32_t omr);
  void predict_branch(const DspBranchDecision &decision, std::uint64_t identity = 0);
  [[nodiscard]] bool
  confirm_branch(std::uint32_t pc, std::uint32_t next_pc, std::uint32_t omr, bool redirected);
  void open_loop(const DspLoopOpening &opening, DspFetchedWord opcode, DspFetchedWord extension);
  [[nodiscard]] bool confirm_loop(DspLoopArchitecture native_result, bool redirected);
  [[nodiscard]] bool confirm_opening(DspLoopArchitecture native_result, bool redirected);

private:
  struct Sequence {
    std::atomic<std::uint64_t> serial{0};
  };

  [[nodiscard]] std::uint64_t next_occurrence();
  [[nodiscard]] std::size_t position(DspFetchedWord captured) const;
  [[nodiscard]] std::size_t branch_position(const DspBranchDecision &decision,
                                            std::uint64_t identity) const;
  void shift_queue();
  void clear_continuation();
  void validate_context_storage(const DspBus &bus, std::uint32_t omr) const;
  DspProgramFetchView state_;
  std::array<std::uint64_t, dsp_program_fetch_capacity> occurrences_{};
  std::array<std::optional<DspLoopFetch::Prediction>, dsp_program_fetch_capacity> predictions_{};
  std::array<std::optional<DspLoopBoundary>, dsp_program_fetch_capacity> boundaries_{};
  std::array<DspFetchedWord, 2> consumed_{};
  std::array<DspFetchOrigin, dsp_program_fetch_capacity> origins_{};
  std::array<DspFetchOrigin, 2> consumed_origins_{};
  std::optional<DspFetchContext> context_;
  std::shared_ptr<Sequence> sequence_;
  std::optional<DspLoopFetch> loop_;
  std::optional<DspLoopBoundary> consumed_end_;
  std::optional<DspLoopOpening> opening_;
  std::optional<DspLoopArchitecture> observed_loop_;
  std::optional<DspBranchDecision> branch_;
  std::uint64_t branch_occurrence_ = 0;
  bool branch_target_captured_ = false;
};
}
