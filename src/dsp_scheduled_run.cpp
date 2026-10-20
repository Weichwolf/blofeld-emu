// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_scheduled_run.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
DspScheduledRun::DspScheduledRun(const DspScheduleInputs inputs, const DspMachine &machine) {
  if (inputs.observation != DspStepObservation::reference_completion_boundary) {
    throw Fault("DSP scheduling requires an explicit nominal completion-observation contract");
  }
  const auto &clock = machine.run_clock();
  if (machine.timing_faulted() || clock.active() ||
      inputs.epoch_ticks > std::numeric_limits<std::uint64_t>::max() - clock.elapsed_ticks()) {
    throw Fault("DSP scheduling requires a healthy representable clock epoch");
  }
  state_.inputs_ = inputs;
  state_.timebase_hz_ = clock.timebase_hz();
  state_.completed_ticks_ = clock.elapsed_ticks();
  state_.completed_cycles_ = clock.native_cycle_units();
  state_.horizon_ = inputs.epoch_ticks + clock.elapsed_ticks();
}

void DspScheduledRun::verify(const DspScheduleCheckpoint &state, const DspMachine &machine) {
  const auto &clock = machine.run_clock();
  if (machine.timing_faulted() || clock.active() || clock.timebase_hz() != state.timebase_hz_ ||
      clock.elapsed_ticks() != state.completed_ticks_ ||
      clock.native_cycle_units() != state.completed_cycles_) {
    throw Fault("DSP scheduler/owner clock state requires coordinated recovery");
  }
  if (state.pending_ && machine.plan_step() != *state.pending_) {
    throw Fault("DSP scheduled instruction changed before its declared completion");
  }
}

std::uint64_t DspScheduledRun::deadline(const DspStepPlan &plan) const {
  const auto remaining = std::numeric_limits<std::uint64_t>::max() - state_.inputs_.epoch_ticks;
  if (plan.start_ticks > remaining || plan.duration_ticks > remaining - plan.start_ticks) {
    throw Fault("DSP scheduled completion exhausted the declared epoch");
  }
  return state_.inputs_.epoch_ticks + plan.start_ticks + plan.duration_ticks;
}

void DspScheduledRun::complete(DspMachine &machine,
                               const std::function<void(const DspScheduledStep &)> &before_step,
                               const DspCompletionObserver &after_step) {
  if (!state_.pending_) { throw Fault("DSP scheduler has no prepared instruction"); }
  const auto plan = *state_.pending_;
  const auto completion = deadline(plan);
  if (state_.steps_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP scheduled step counter exhausted");
  }
  if (before_step) { before_step({.completion_ticks = completion, .plan = plan}); }
  verify(state_, machine);
  machine.cpu().step();
  const auto &clock = machine.run_clock();
  if (clock.elapsed_ticks() != plan.start_ticks + plan.duration_ticks ||
      clock.native_cycle_units() != plan.before.native_cycles + plan.native_cycle_units) {
    throw Fault("DSP completed duration differs from its native instruction preview");
  }
  state_.completed_ticks_ = clock.elapsed_ticks();
  state_.completed_cycles_ = clock.native_cycle_units();
  ++state_.steps_;
  state_.pending_.reset();
  notify_completion(machine, {.completion_ticks = completion, .plan = plan}, after_step);
}

void DspScheduledRun::notify_completion(DspMachine &machine,
                                        const DspScheduledStep &step,
                                        const DspCompletionObserver &observer) {
  const auto observe = observer.observe;
  if (!observe) { return; }
  const auto expected = machine.cpu().debug_state();
  try {
    observe(step);
    verify(state_, machine);
    if (machine.cpu().debug_state() != expected) {
      throw Fault("Post-completion callback changed the native CPU state");
    }
  } catch (const std::exception &error) {
    throw Fault("DSP post-completion observation at=" + std::to_string(step.completion_ticks) +
                " completed-steps=" + std::to_string(state_.steps_) + ": " + error.what());
  }
}

unsigned
DspScheduledRun::advance_to(DspMachine &machine,
                            const std::uint64_t horizon,
                            const DspScheduleBoundary boundary,
                            const unsigned maximum_steps,
                            const std::function<void(const DspScheduledStep &)> &before_step,
                            const DspCompletionObserver &after_step) {
  if (active_ || faulted_) { throw Fault("DSP scheduler requires a healthy inactive boundary"); }
  active_ = true;
  unsigned completed = 0;
  std::optional<std::uint64_t> attempted_completion;
  try {
    if (boundary != DspScheduleBoundary::before && boundary != DspScheduleBoundary::through) {
      throw Fault("Unknown DSP schedule boundary policy");
    }
    if (horizon < state_.horizon_ ||
        (horizon == state_.horizon_ && state_.boundary_ == DspScheduleBoundary::through &&
         boundary == DspScheduleBoundary::before)) {
      throw Fault("DSP schedule horizon moved backwards");
    }
    verify(state_, machine);
    while (state_.inputs_.epoch_ticks + state_.completed_ticks_ < horizon || state_.pending_) {
      if (!state_.pending_) { state_.pending_ = machine.plan_step(); }
      const auto end = deadline(*state_.pending_);
      if (end > horizon || (end == horizon && boundary == DspScheduleBoundary::before)) { break; }
      if (completed == maximum_steps) { throw Fault("DSP scheduled instruction budget exhausted"); }
      attempted_completion = end;
      complete(machine, before_step, after_step);
      attempted_completion.reset();
      ++completed;
    }
    state_.horizon_ = horizon;
    state_.boundary_ = boundary;
    active_ = false;
    return completed;
  } catch (const std::exception &error) {
    active_ = false;
    faulted_ = true;
    throw Fault("DSP scheduled completion pc=" + hex(machine.cpu().state().pc, 6) +
                " horizon=" + std::to_string(horizon) + " attempted-completion=" +
                (attempted_completion ? std::to_string(*attempted_completion) : "none") +
                " completed-reference-ticks=" + std::to_string(state_.completed_ticks_) +
                " completed-native-cycle-units=" + std::to_string(state_.completed_cycles_) + ": " +
                error.what());
  } catch (...) {
    active_ = false;
    faulted_ = true;
    throw;
  }
}

DspScheduleCheckpoint DspScheduledRun::checkpoint(const DspMachine &machine) const {
  if (active_ || faulted_) { throw Fault("DSP schedule checkpoint requires a healthy boundary"); }
  verify(state_, machine);
  return state_;
}

std::uint64_t DspScheduledRun::next_completion_ticks(const DspMachine &machine) const {
  if (active_ || faulted_) { throw Fault("DSP completion preview requires a healthy boundary"); }
  verify(state_, machine);
  return deadline(state_.pending_ ? *state_.pending_ : machine.plan_step());
}

void DspScheduledRun::restore(const DspScheduleCheckpoint &checkpoint, const DspMachine &machine) {
  if (active_) { throw Fault("Cannot restore an active DSP schedule"); }
  if (checkpoint.inputs_ != state_.inputs_ || checkpoint.timebase_hz_ != state_.timebase_hz_) {
    throw Fault("DSP schedule checkpoint has an incompatible observation/epoch contract");
  }
  verify(checkpoint, machine);
  state_ = checkpoint;
  faulted_ = false;
}

}
