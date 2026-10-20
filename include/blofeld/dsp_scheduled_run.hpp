// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_machine.hpp"

#include <cstdint>
#include <functional>
#include <optional>

namespace blofeld {

enum class DspStepObservation : std::uint8_t { unqualified, reference_completion_boundary };
enum class DspScheduleBoundary : std::uint8_t { before, through };

struct DspScheduleInputs {
  std::uint64_t epoch_ticks = 0;
  DspStepObservation observation = DspStepObservation::unqualified;
  bool operator==(const DspScheduleInputs &) const = default;
};

class DspScheduleCheckpoint {
public:
  DspScheduleCheckpoint(const DspScheduleCheckpoint &) = default;
  DspScheduleCheckpoint &operator=(const DspScheduleCheckpoint &) = default;

private:
  friend class DspScheduledRun;
  DspScheduleCheckpoint() = default;
  DspScheduleInputs inputs_;
  std::uint64_t timebase_hz_ = 0;
  std::uint64_t horizon_ = 0;
  std::uint64_t completed_ticks_ = 0;
  std::uint64_t completed_cycles_ = 0;
  std::uint64_t steps_ = 0;
  DspScheduleBoundary boundary_ = DspScheduleBoundary::through;
  std::optional<DspStepPlan> pending_;
};

struct DspScheduledStep {
  std::uint64_t completion_ticks = 0;
  DspStepPlan plan;
};

// Distinct from the pre-step callback: runs only after native CPU/clock effects
// and completed-step accounting commit. Callback failures quarantine the schedule.
struct DspCompletionObserver {
  std::function<void(const DspScheduledStep &)> observe;
};

// Single-threaded nominal completion experiment. Instruction reads/effects are
// observed at their declared completion boundary, not physical pipeline phases.
// No MCU reference, pin policy or external clock advance is retained here.
class DspScheduledRun {
public:
  DspScheduledRun(DspScheduleInputs inputs, const DspMachine &machine);
  DspScheduledRun(const DspScheduledRun &) = delete;
  DspScheduledRun &operator=(const DspScheduledRun &) = delete;
  unsigned advance_to(DspMachine &machine,
                      std::uint64_t horizon,
                      DspScheduleBoundary boundary,
                      unsigned maximum_steps,
                      const std::function<void(const DspScheduledStep &)> &before_step = {},
                      const DspCompletionObserver &after_step = {});
  [[nodiscard]] DspScheduleCheckpoint checkpoint(const DspMachine &machine) const;
  [[nodiscard]] std::uint64_t next_completion_ticks(const DspMachine &machine) const;
  void restore(const DspScheduleCheckpoint &checkpoint, const DspMachine &machine);

  [[nodiscard]] std::uint64_t horizon_ticks() const { return state_.horizon_; }

  [[nodiscard]] std::uint64_t completed_steps() const { return state_.steps_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  static void verify(const DspScheduleCheckpoint &state, const DspMachine &machine);
  [[nodiscard]] std::uint64_t deadline(const DspStepPlan &plan) const;
  void complete(DspMachine &machine,
                const std::function<void(const DspScheduledStep &)> &before_step,
                const DspCompletionObserver &after_step);
  void notify_completion(DspMachine &machine,
                         const DspScheduledStep &step,
                         const DspCompletionObserver &observer);
  DspScheduleCheckpoint state_;
  bool active_ = false;
  bool faulted_ = false;
};

}
