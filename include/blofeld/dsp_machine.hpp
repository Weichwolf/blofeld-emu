// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dax.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/dsp_run_clock.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {

enum class DspRunTiming : std::uint8_t { unqualified, native_internal_estimates };
enum class DspInterruptTiming : std::uint8_t { unqualified, native_vector_estimates };

struct DspRunContract {
  DspRunClockInputs clock;
  DspRunTiming timing = DspRunTiming::unqualified;
  DspInterruptTiming interrupts = DspInterruptTiming::unqualified;
  bool operator==(const DspRunContract &) const = default;
};

struct DspStepPlan {
  DspDebugState before;
  unsigned native_cycle_units = 0;
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
  // Nominal completion preview only; no instruction, peripheral or clock advance.
  [[nodiscard]] DspStepPlan plan_step() const;
  // Untimed owner-controlled access suspension; no automatic IRQ entry or clocking.
  [[nodiscard]] DspReferenceProgress begin_reference_instruction(DspReferenceExecution contract);
  // Explicit untimed interrupt entry, including both vector fetches and acknowledgement.
  [[nodiscard]] DspReferenceProgress begin_reference_interrupt(DspReferenceExecution contract);
  [[nodiscard]] DspReferenceProgress advance_reference_access(const DspReferenceTicket &ticket);
  [[nodiscard]] std::optional<DspReferencePending> reference_pending() const;
  [[nodiscard]] std::optional<DspDmaWord> advance_dma(DspDmaInputs inputs);
  [[nodiscard]] DspDmaBusResult advance_dma(DspDmaInputs inputs, DspCoreBusUse core);
  void observe_dax(DspDaxInputs inputs);
  [[nodiscard]] DspCheckpoint checkpoint() const;
  // Strong exception guarantee; a prior valid checkpoint can recover a fault.
  void restore(const DspCheckpoint &checkpoint);

private:
  friend class DspScheduledRun;
  void check_admission_contract() const;
  void verify_prepared_step(const DspStepPlan &plan) const;
  void execute_prepared_step(const DspStepPlan &plan);
  struct State;
  std::unique_ptr<State> state_;
};

}
