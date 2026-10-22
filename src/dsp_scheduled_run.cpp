// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_scheduled_run.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
namespace {
std::optional<std::uint64_t> earlier_deadline(const std::optional<std::uint64_t> native,
                                              const std::optional<std::uint64_t> external) {
  if (!native) { return external; }
  if (!external) { return native; }
  return std::min(*native, *external);
}
}

DspScheduledRun::DspScheduledRun(const DspScheduleInputs inputs, const DspMachine &machine) {
  if (inputs.observation != DspStepObservation::reference_completion_boundary) {
    throw Fault("DSP scheduling requires an explicit nominal completion-observation contract");
  }
  if (inputs.irq_admission != DspIrqAdmission::reject_changes &&
      inputs.irq_admission != DspIrqAdmission::reference_prepare_boundary) {
    throw Fault("Unknown DSP schedule interrupt admission contract");
  }
  if (inputs.irq_admission == DspIrqAdmission::reference_prepare_boundary) {
    machine.check_admission_contract();
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
  state_.native_epoch_ = clock.native_epoch();
  state_.horizon_ = inputs.epoch_ticks + clock.elapsed_ticks();
}

void DspScheduledRun::verify(const DspScheduleCheckpoint &state, const DspMachine &machine) {
  const auto &clock = machine.run_clock();
  if (machine.timing_faulted() || clock.active() || clock.timebase_hz() != state.timebase_hz_ ||
      clock.elapsed_ticks() != state.completed_ticks_ ||
      clock.native_cycle_units() != state.completed_cycles_ ||
      clock.native_epoch() != state.native_epoch_) {
    throw Fault("DSP scheduler/owner clock state requires coordinated recovery");
  }
  if (state.pending_) {
    if (state.inputs_.irq_admission == DspIrqAdmission::reference_prepare_boundary) {
      machine.verify_prepared_step(*state.pending_);
    } else if (machine.plan_step() != *state.pending_) {
      throw Fault("DSP scheduled instruction changed before its declared completion");
    }
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
  if (state_.inputs_.irq_admission == DspIrqAdmission::reference_prepare_boundary) {
    machine.execute_prepared_step(plan);
  } else {
    machine.cpu().step();
  }
  const auto &clock = machine.run_clock();
  if (clock.elapsed_ticks() != plan.start_ticks + plan.duration_ticks ||
      clock.native_cycle_units() != state_.completed_cycles_ + plan.native_cycle_units) {
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
                            const DspCompletionObserver &after_step,
                            const DspExternalEvents &external) {
  if (active_ || faulted_) { throw Fault("DSP scheduler requires a healthy inactive boundary"); }
  active_ = true;
  unsigned completed = 0;
  std::optional<std::uint64_t> attempted_completion;
  std::optional<std::uint64_t> attempted_external;
  try {
    check_horizon(horizon, boundary);
    verify(state_, machine);
    bind_external(machine, external);
    unsigned external_count = 0;
    while (true) {
      const bool boundary_input =
          state_.inputs_.irq_admission == DspIrqAdmission::reference_prepare_boundary &&
          !state_.pending_ && state_.next_external_ &&
          *state_.next_external_ == state_.inputs_.epoch_ticks + state_.completed_ticks_;
      const auto native_end = boundary_input ? std::nullopt : prepare_native(machine, horizon);
      const auto next = earlier_deadline(native_end, state_.next_external_);
      if (!next) { break; }
      const auto end = *next;
      if (end > horizon || (end == horizon && boundary == DspScheduleBoundary::before)) { break; }
      if (external_first(native_end)) {
        if (external_count == external.maximum_events) {
          throw Fault("DSP scheduled external event budget exhausted");
        }
        attempted_external = end;
        complete_external(machine, external);
        attempted_external.reset();
        ++external_count;
        continue;
      }
      if (completed == maximum_steps) { throw Fault("DSP scheduled instruction budget exhausted"); }
      attempted_completion = end;
      complete(machine, before_step, after_step);
      attempted_completion.reset();
      ++completed;
      if (state_.external_bound_) { bind_external(machine, external); }
    }
    state_.horizon_ = horizon;
    state_.boundary_ = boundary;
    state_.advanced_ = true;
    active_ = false;
    return completed;
  } catch (const std::exception &error) {
    active_ = false;
    faulted_ = true;
    throw Fault(failure_context(machine, horizon, attempted_completion, attempted_external) + ": " +
                error.what());
  } catch (...) {
    active_ = false;
    faulted_ = true;
    throw;
  }
}

void DspScheduledRun::check_horizon(const std::uint64_t horizon,
                                    const DspScheduleBoundary boundary) const {
  if (boundary != DspScheduleBoundary::before && boundary != DspScheduleBoundary::through) {
    throw Fault("Unknown DSP schedule boundary policy");
  }
  if (horizon < state_.horizon_ ||
      (horizon == state_.horizon_ && state_.boundary_ == DspScheduleBoundary::through &&
       boundary == DspScheduleBoundary::before)) {
    throw Fault("DSP schedule horizon moved backwards");
  }
}

std::optional<std::uint64_t> DspScheduledRun::prepare_native(DspMachine &machine,
                                                             const std::uint64_t horizon) {
  if (state_.inputs_.epoch_ticks + state_.completed_ticks_ >= horizon && !state_.pending_) {
    return std::nullopt;
  }
  if (!state_.pending_) { state_.pending_ = machine.plan_step(); }
  return deadline(*state_.pending_);
}

std::string DspScheduledRun::failure_context(const DspMachine &machine,
                                             const std::uint64_t horizon,
                                             const std::optional<std::uint64_t> native,
                                             const std::optional<std::uint64_t> external) const {
  auto result = "DSP scheduled completion pc=" + hex(machine.cpu().state().pc, 6) +
                " horizon=" + std::to_string(horizon) +
                " attempted-completion=" + (native ? std::to_string(*native) : "none") +
                " completed-reference-ticks=" + std::to_string(state_.completed_ticks_) +
                " completed-native-cycle-units=" + std::to_string(state_.completed_cycles_);
  if (state_.external_bound_) {
    result += " attempted-external=" + (external ? std::to_string(*external) : "none") +
              " completed-external-events=" + std::to_string(state_.external_events_);
  }
  return result;
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

std::uint64_t DspScheduledRun::next_event_ticks(const DspMachine &machine) const {
  const auto native = next_completion_ticks(machine);
  return state_.next_external_ ? std::min(native, *state_.next_external_) : native;
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
