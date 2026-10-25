// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {

enum class DspRunTiming : std::uint8_t {
  unqualified,
  native_internal_estimates,
  native_internal_agu_interlocks,
  native_interlock_estimates
};
enum class DspInterruptTiming : std::uint8_t { unqualified, native_vector_estimates };
enum class DspResetTiming : std::uint8_t {
  unqualified,
  reference_instruction_boundary,
  reference_extal_hold
};

struct DspHardwareResetInputs {
  std::uint8_t mode_known = 0;
  std::uint8_t mode_levels = 0;
  std::optional<bool> pinit;
  std::uint32_t recovery_cycles = 0;
};

struct DspRunContract {
  DspRunClockInputs clock;
  DspRunTiming timing = DspRunTiming::unqualified;
  DspInterruptTiming interrupts = DspInterruptTiming::unqualified;
  DspResetTiming resets = DspResetTiming::unqualified;
  bool operator==(const DspRunContract &) const = default;
};

struct DspStepPlan {
  DspDebugState before;
  unsigned native_cycle_units = 0;
  unsigned interlock_cycle_units = 0;
  std::uint64_t start_ticks = 0;
  std::uint64_t duration_ticks = 0;
  std::optional<DspInterruptVector> interrupt;
  bool operator==(const DspStepPlan &) const = default;
};

// Immutable trusted in-process state; not a portable file or whole-board format.
class DspCheckpoint {
public:
  DspCheckpoint(const DspCheckpoint &) = default;
  DspCheckpoint &operator=(const DspCheckpoint &) = default;
  DspCheckpoint(DspCheckpoint &&) noexcept = default;
  DspCheckpoint &operator=(DspCheckpoint &&) noexcept = default;
  ~DspCheckpoint();

private:
  friend class DspMachine;
  struct Storage;
  explicit DspCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Single-threaded DSP owner. Restore replaces bus/CPU objects and invalidates
// references to them; external wiring and schedules belong to their callers.
class DspMachine {
public:
  DspMachine();
  explicit DspMachine(std::optional<DspRunContract> run);
  ~DspMachine();
  DspMachine(const DspMachine &) = delete;
  DspMachine &operator=(const DspMachine &) = delete;
  [[nodiscard]] DspBus &bus();
  [[nodiscard]] DspCore &cpu();
  [[nodiscard]] const DspBus &bus() const;
  [[nodiscard]] const DspCore &cpu() const;
  [[nodiscard]] bool clocked() const;
  [[nodiscard]] const DspRunClock &run_clock() const;
  [[nodiscard]] bool timing_faulted() const;
  // All mode/PINIT pins stay stable through assertion and the supplied recovery.
  // Only observed mode5/PINIT-low is qualified; assertion starts at a completed step.
  void begin_hardware_reset(DspHardwareResetInputs inputs);
  std::uint64_t advance_hardware_reset(std::uint64_t maximum_extal_cycles);
  std::uint64_t advance_hardware_reset_ticks(std::uint64_t maximum_ticks);
  void release_hardware_reset();
  [[nodiscard]] bool hardware_reset_active() const;
  // Nominal completion preview only; no instruction, peripheral or clock advance.
  [[nodiscard]] DspStepPlan plan_step() const;
  // Untimed owner-controlled access suspension; no automatic IRQ entry or clocking.
  [[nodiscard]] DspReferenceProgress begin_reference_instruction(DspReferenceExecution contract);
  // Untimed two-candidate ownership; not hardware operand forwarding/stage timing.
  [[nodiscard]] DspReferenceProgress
  overlap_reference_instruction(const DspReferenceTicket &ticket);
  [[nodiscard]] std::optional<DspReferencePending> reference_retirement() const;
  [[nodiscard]] DspInstructionRetire retire_reference_overlap(const DspReferenceTicket &ticket);
  // Explicit untimed interrupt entry, including both vector fetches and acknowledgement.
  [[nodiscard]] DspReferenceProgress begin_reference_interrupt(DspReferenceExecution contract);
  // Capture the pending opcode/extension now; keep native decoding suspended.
  [[nodiscard]] DspReferencePending latch_reference_program(const DspReferenceTicket &ticket);
  // Explicit untimed F1/F2 transitions; sampled words feed real native admissions.
  [[nodiscard]] DspProgramFetchView start_reference_fetch(const DspReferenceTicket &ticket);
  [[nodiscard]] DspProgramFetchView advance_reference_fetch(DspProgramFetchStage stage);
  [[nodiscard]] DspProgramFetchView reference_fetch() const;
  [[nodiscard]] DspReferenceProgress advance_reference_access(const DspReferenceTicket &ticket);
  // Adjudicate actual pending memory effects; hold the native ticket on DMA stalls.
  // Reference samples only: neither native pipeline phases nor clocks are supplied.
  [[nodiscard]] DspReferenceDmaProgress advance_reference_dma(DspDmaInputs inputs,
                                                              const DspReferenceTicket &ticket);
  [[nodiscard]] std::optional<DspReferencePending> reference_pending() const;
  // Derive occupancy from actual selected F2/native effects; honor DMA stalls.
  [[nodiscard]] DspReferenceCycleResult
  advance_reference_bus_cycle(const DspReferenceCycleInputs &inputs);
  [[nodiscard]] std::optional<DspDmaWord> advance_dma(DspDmaInputs inputs);
  [[nodiscard]] DspDmaBusResult advance_dma(DspDmaInputs inputs, DspCoreBusUse core);
  void observe_dax(DspDaxInputs inputs);
  [[nodiscard]] DspCheckpoint checkpoint() const;
  // Strong exception guarantee; a prior valid checkpoint can recover a fault.
  void restore(const DspCheckpoint &checkpoint);

private:
  friend class DspScheduledRun;
  void require_running() const;
  friend class DspFetchRun;
  friend struct DspFetchState;
  friend struct DspInstructionPipeline;
  [[nodiscard]] DspAddressStageTicket capture_reference_address(std::uint32_t address);
  [[nodiscard]] DspAddressStageView address_stage(const DspAddressStageTicket &ticket) const;
  [[nodiscard]] DspAddressStageTicket
  rebind_address_stage(const DspAddressStageTicket &ticket) const;
  void validate_reference_overlap(const DspReferenceTicket &ticket) const;
  [[nodiscard]] std::shared_ptr<const bool> reference_clock_owner() const;
  void fail_reference_clock() noexcept;
  void check_admission_contract() const;
  void verify_prepared_step(const DspStepPlan &plan) const;
  void execute_prepared_step(const DspStepPlan &plan);
  struct State;
  std::unique_ptr<State> state_;
};

}
