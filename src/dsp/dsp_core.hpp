// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {
struct DspFetchedWord;
struct DspAddressRecord;
class DspAddressStageTicket;
class DspPeripheralReadTicket;
struct DspPeripheralReadPoint;
struct DspBranchDecision;
struct DspAddressStageView;
struct DspMemoryStageResult;

struct DspState {
  std::uint32_t pc = 0;
  std::uint32_t sr = 0;
  std::uint32_t omr = 0;
  std::uint32_t sp = 0;
  std::uint32_t vba = 0;
  std::uint32_t stack_pc = 0;
  std::uint32_t stack_sr = 0;
  std::uint32_t la = 0;
  std::uint32_t lc = 0;
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::uint64_t a = 0;
  std::uint64_t b = 0;
  std::array<std::uint32_t, 8> r{};
  std::array<std::uint32_t, 8> n{};
  std::array<std::uint32_t, 8> m{};
  bool operator==(const DspState &) const = default;
};

// Observations without destructive SSH reads. This view omits native REP/IRQ
// continuation; DspMachine provides opaque restorable checkpoints.
struct DspDebugState {
  DspState cpu;
  // SDK diagnostic counters, not calibrated elapsed instruction/bus clocks.
  std::uint64_t native_instructions = 0;
  std::uint64_t native_cycles = 0;
  std::uint32_t ep = 0;
  std::uint32_t sz = 0;
  std::uint32_t sc = 0;
  std::array<std::uint32_t, 16> stack_high{};
  std::array<std::uint32_t, 16> stack_low{};
  bool operator==(const DspDebugState &) const = default;
};

// Both real vector words are required, including a word not executed after JSR.
// Pure preview identity; neither a fetch nor an interrupt acknowledgement.
struct DspInterruptVector {
  DspInterruptRequest request;
  std::uint32_t address = 0;
  std::array<std::uint32_t, 2> words{};
  bool operator==(const DspInterruptVector &) const = default;
};

// Opaque native state; only DspMachine can pair it with matching bus contents.
class DspCoreCheckpoint {
public:
  DspCoreCheckpoint(const DspCoreCheckpoint &) = default;
  DspCoreCheckpoint &operator=(const DspCoreCheckpoint &) = default;
  DspCoreCheckpoint(DspCoreCheckpoint &&) noexcept = default;
  DspCoreCheckpoint &operator=(DspCoreCheckpoint &&) noexcept = default;
  ~DspCoreCheckpoint();

private:
  friend class DspCore;
  struct Storage;
  explicit DspCoreCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Native interpreter reference path. Each DO/REP body instruction can yield
// independently. Interrupt entry executes at most two vector words atomically;
// peripheral time remains an explicit board-model input.
class DspCore {
public:
  explicit DspCore(DspBus &bus);
  ~DspCore();
  DspCore(const DspCore &) = delete;
  DspCore &operator=(const DspCore &) = delete;
  void reset_spi_boot();
  [[nodiscard]] DspState state() const;
  [[nodiscard]] DspDebugState debug_state() const;
  void step();
  void step(DspNativeAccessObserver observer);

private:
  friend class DspMachine;

  struct RunHooks {
    std::function<void()> ready;
    std::function<void(DspState, bool, unsigned, std::optional<std::uint32_t>)> begin;
    std::function<void(unsigned)> complete;
    std::function<void()> failed;
    std::function<void()> reset;
    std::function<void(std::uint32_t)> fetched;
    std::function<void(DspMemoryAccess)> data;
    bool vector_estimates = false;
    bool timing_opcode = false;
  };

  struct StepEstimate {
    unsigned units = 0;
    std::optional<std::uint32_t> opcode = std::nullopt;
    std::optional<DspInterruptVector> interrupt;
  };

  struct Admission {
    std::optional<DspInterruptRequest> interrupt = std::nullopt;
  };

  void set_run_hooks(RunHooks hooks);
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] std::uint64_t native_cycles() const;
  [[nodiscard]] StepEstimate next_step_cycles(bool vector_estimates,
                                              const Admission *admission = nullptr,
                                              bool timing_opcode = false) const;
  void step_admitted(std::optional<DspInterruptRequest> interrupt);
  void step_impl(DspNativeAccessObserver observer, const Admission *admission);
  [[nodiscard]] DspCoreCheckpoint checkpoint() const;
  void restore_checkpoint(const DspCoreCheckpoint &checkpoint);
  [[nodiscard]] bool executing() const;
  [[nodiscard]] DspReferenceProgress begin_reference_instruction(DspReferenceExecution contract);
  [[nodiscard]] DspReferenceProgress
  overlap_reference_instruction(const DspReferenceTicket &ticket);
  void validate_reference_overlap(const DspReferenceTicket &ticket) const;
  [[nodiscard]] std::optional<DspReferencePending> reference_retirement() const;
  void validate_reference_retirement(const DspReferenceTicket &ticket) const;
  [[nodiscard]] DspInstructionRetire retire_reference_overlap(const DspReferenceTicket &ticket);
  [[nodiscard]] DspReferenceProgress begin_reference_interrupt(DspReferenceExecution contract);
  [[nodiscard]] DspReferenceProgress
  begin_reference_step(DspReferenceExecution contract,
                       bool interrupt_entry,
                       const DspAddressRecord *address = nullptr);
  [[nodiscard]] DspReferenceProgress advance_reference_access(const DspReferenceTicket &ticket);
  [[nodiscard]] DspReferencePending latch_reference_program(const DspReferenceTicket &ticket);
  [[nodiscard]] DspProgramFetchView start_reference_fetch(const DspReferenceTicket &ticket);
  [[nodiscard]] DspProgramFetchView advance_reference_fetch(DspProgramFetchStage stage);
  [[nodiscard]] DspProgramFetchView reference_fetch() const;
  [[nodiscard]] std::optional<DspFetchedWord> pipeline_fetched_word(std::size_t index) const;
  // Read-only native-owned loop proofs and actual occurrence metadata.
  [[nodiscard]] bool pipeline_loop_control() const;
  [[nodiscard]] bool pipeline_loop_result() const;
  [[nodiscard]] bool pipeline_interrupt_deferred() const;
  [[nodiscard]] std::optional<DspRepeatControl> pipeline_repeat_control() const;
  [[nodiscard]] std::optional<DspRepeatControl> pipeline_repeat_result() const;
  [[nodiscard]] std::optional<DspFetchedWord> pipeline_repeat_body() const;
  [[nodiscard]] std::optional<std::uint32_t> pipeline_loop_last(DspFetchedWord word) const;
  [[nodiscard]] std::uint32_t pipeline_continuation(DspFetchedWord word, unsigned words) const;
  void hold_pipeline_loop(DspFetchedWord word);
  void hold_pipeline_branch(DspFetchedWord word);

  void validate_reference_ticket(const DspReferenceTicket &ticket) const;
  void validate_reference_admission(const DspReferenceTicket &ticket,
                                    bool retiring_previous = false,
                                    std::optional<std::uint64_t> cycle = std::nullopt) const;
  void validate_reference_fetch(DspProgramFetchStage stage) const;
  void fail_reference_cycle() noexcept;
  [[nodiscard]] std::optional<DspReferencePending> reference_pending() const;
  [[nodiscard]] std::optional<DspReferencePending> reference_pending_unchecked() const;
  [[nodiscard]] DspReferenceProgress
  reference_progress(std::optional<DspReferenceEffect> completed = std::nullopt) const;
  [[nodiscard]] DspBranchDecision capture_reference_branch(std::uint32_t address,
                                                           std::uint64_t occurrence = 0);
  [[nodiscard]] DspAddressStageTicket capture_reference_address(std::uint32_t address,
                                                                std::uint64_t occurrence = 0);
  [[nodiscard]] DspAddressStageView address_stage(const DspAddressStageTicket &ticket) const;
  [[nodiscard]] DspAddressStageTicket
  rebind_address_stage(const DspAddressStageTicket &ticket) const;
  [[nodiscard]] DspAddressStageView publish_reference_address(const DspAddressStageTicket &ticket);
  void validate_reference_address(const DspAddressStageTicket &ticket, bool published) const;
  [[nodiscard]] DspMemoryStageResult execute_reference_memory(const DspAddressStageTicket &ticket);
  [[nodiscard]] DspPeripheralReadTicket
  capture_reference_peripheral_read(DspPeripheralReadPoint point);
  [[nodiscard]] DspPeripheralReadTicket
  rebind_reference_peripheral_read(const DspPeripheralReadTicket &ticket) const;
  [[nodiscard]] DspReferenceProgress
  advance_reference_peripheral_read(const DspPeripheralReadTicket &read,
                                    const DspReferenceTicket &native);
  void validate_reference_peripheral_read(const DspPeripheralReadTicket &ticket) const;
  [[nodiscard]] DspMemoryAccess pipeline_peripheral_access(std::uint32_t address,
                                                           std::uint64_t occurrence) const;
  [[nodiscard]] DspPeripheralReadTicket
  capture_pipeline_peripheral_read(DspPeripheralReadPoint point, std::uint64_t occurrence);
  [[nodiscard]] DspBranchDecision
  preview_pipeline_peripheral_branch(const DspPeripheralReadTicket &read);
  [[nodiscard]] DspReferenceProgress
  advance_pipeline_peripheral_read(const DspPeripheralReadTicket &read,
                                   const DspReferenceTicket &native);
  [[nodiscard]] DspPeripheralReadTicket capture_peripheral_read(DspPeripheralReadPoint point,
                                                                bool automatic,
                                                                std::uint64_t occurrence = 0);
  [[nodiscard]] DspReferenceProgress advance_peripheral_read(const DspPeripheralReadTicket &read,
                                                             const DspReferenceTicket &native,
                                                             DspReferenceExecution execution);
  [[nodiscard]] bool has_reference_peripheral_reads() const;
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
