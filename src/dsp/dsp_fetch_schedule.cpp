// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_fetch_schedule.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
struct DspFetchScheduleCheckpoint::Storage {
  DspFetchScheduleInputs inputs;
  DspFetchScheduleState state;
  DspFetchCheckpoint native;
  const DspMachine *owner;
  std::weak_ptr<const bool> lifetime;
};

DspFetchScheduleCheckpoint::DspFetchScheduleCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspFetchScheduleCheckpoint::~DspFetchScheduleCheckpoint() = default;

class DspFetchScheduledRun::Guard {
public:
  explicit Guard(DspFetchScheduledRun &owner)
      : owner_(owner), exceptions_(std::uncaught_exceptions()) {
    owner_.active_ = true;
  }

  ~Guard() {
    owner_.active_ = false;
    if (std::uncaught_exceptions() > exceptions_) { owner_.faulted_ = true; }
  }

  Guard(const Guard &) = delete;
  Guard &operator=(const Guard &) = delete;

private:
  DspFetchScheduledRun &owner_;
  int exceptions_;
};

DspFetchScheduledRun::DspFetchScheduledRun(const DspFetchScheduleInputs inputs, DspMachine &machine)
    : DspFetchScheduledRun(inputs, machine, std::nullopt) {}

DspFetchScheduledRun::DspFetchScheduledRun(
    const DspFetchScheduleInputs inputs,
    DspMachine &machine,
    const std::optional<DspHardwareResetInputs> initial_reset)
    : inputs_(inputs), machine_(machine), run_(inputs.dsp, machine, initial_reset) {
  state_.horizon.ticks = inputs.initial_horizon_ticks.value_or(inputs.epoch_ticks);
  if (state_.horizon.ticks > inputs.epoch_ticks) {
    throw Fault("DSP pipeline initial horizon cannot skip native work before its epoch");
  }
  static_cast<void>(native_deadline());
}

const DspFetchScheduleCheckpoint::Storage &
DspFetchScheduledRun::clone_source(const DspFetchScheduleCheckpoint &checkpoint,
                                   const DspMachine &target) {
  if (!checkpoint.storage_ ||
      (checkpoint.storage_->lifetime.lock() && checkpoint.storage_->owner == &target)) {
    throw Fault("DSP timeline clone requires a trusted checkpoint and distinct isolated target");
  }
  return *checkpoint.storage_;
}

DspFetchScheduledRun::DspFetchScheduledRun(const DspFetchScheduleCheckpoint &checkpoint,
                                           DspMachine &target)
    : DspFetchScheduledRun(clone_source(checkpoint, target), target) {}

DspFetchScheduledRun::DspFetchScheduledRun(const DspFetchScheduleCheckpoint::Storage &source,
                                           DspMachine &target)
    : inputs_(source.inputs), machine_(target), run_(source.native, target), state_(source.state) {
  run_.validate_boundary(machine_);
  static_cast<void>(native_deadline());
}

bool DspFetchScheduledRun::faulted() const {
  return faulted_ || run_.faulted();
}

const DspFetchRun &DspFetchScheduledRun::run() const {
  return run_;
}

DspFetchScheduleState DspFetchScheduledRun::state() const {
  return state_;
}

void DspFetchScheduledRun::healthy() const {
  if (active_ || faulted()) {
    throw Fault("DSP pipeline schedule requires a healthy inactive owner");
  }
  run_.validate_boundary(machine_);
}

std::uint64_t DspFetchScheduledRun::absolute(const std::uint64_t relative) const {
  if (relative > std::numeric_limits<std::uint64_t>::max() - inputs_.epoch_ticks) {
    throw Fault("DSP pipeline schedule exhausted absolute time");
  }
  return inputs_.epoch_ticks + relative;
}

std::optional<std::uint64_t> DspFetchScheduledRun::native_deadline() const {
  const auto elapsed = run_.elapsed_ticks();
  const auto duration = run_.hardware_reset_active()
                            ? run_.hardware_reset_remaining_ticks()
                            : std::optional{run_.next_cycle_ticks(machine_)};
  if (!duration) { return std::nullopt; }
  if (*duration > std::numeric_limits<std::uint64_t>::max() - elapsed) {
    throw Fault("DSP pipeline schedule exhausted relative time");
  }
  return absolute(elapsed + *duration);
}

void DspFetchScheduledRun::check_horizon(const DspFetchHorizon horizon) const {
  const auto valid = [](const DspScheduleBoundary boundary) {
    return boundary == DspScheduleBoundary::before || boundary == DspScheduleBoundary::through;
  };
  if (!valid(horizon.native) || !valid(horizon.external) || horizon.ticks < state_.horizon.ticks ||
      (state_.advanced && horizon.ticks == state_.horizon.ticks &&
       ((state_.horizon.native == DspScheduleBoundary::through &&
         horizon.native == DspScheduleBoundary::before) ||
        (state_.horizon.external == DspScheduleBoundary::through &&
         horizon.external == DspScheduleBoundary::before)))) {
    throw Fault("DSP pipeline horizon cannot rewind time or a consumed boundary");
  }
}

bool DspFetchScheduledRun::eligible(const std::uint64_t ticks,
                                    const DspScheduleBoundary boundary,
                                    const DspFetchHorizon horizon) {
  return ticks < horizon.ticks ||
         (ticks == horizon.ticks && boundary == DspScheduleBoundary::through);
}

bool DspFetchScheduledRun::external_first(const std::optional<std::uint64_t> native) const {
  if (!state_.next_external) { return false; }
  if (!native || *state_.next_external < *native) { return true; }
  if (*state_.next_external > *native) { return false; }
  if (state_.order == DspExternalOrder::reject_tie) {
    throw Fault("Unqualified simultaneous pipeline period and external event");
  }
  return state_.order == DspExternalOrder::external_before_dsp;
}

DspFetchScheduledRun::Step DspFetchScheduledRun::next_step(const DspFetchHorizon horizon) const {
  const auto native = native_deadline();
  const auto event = state_.next_external;
  const bool native_due = native && eligible(*native, horizon.native, horizon);
  const bool event_due = event && eligible(*event, horizon.external, horizon);
  if (!native_due && !event_due) { return Step::none; }
  const bool first = external_first(native);
  if (first ? !event_due : !native_due) {
    throw Fault("Pipeline horizon contradicts its declared event tie order");
  }
  return first ? Step::external : Step::native;
}

void DspFetchScheduledRun::reset_until(const std::uint64_t ticks, const bool include_recovery) {
  if (!run_.hardware_reset_active()) { return; }
  const auto now = absolute(run_.elapsed_ticks());
  if (ticks < now) { throw Fault("DSP pipeline reset time cannot rewind"); }
  auto duration = ticks - now;
  const auto remaining = run_.hardware_reset_remaining_ticks();
  if (!include_recovery && remaining && duration >= *remaining) { duration = *remaining - 1; }
  if (duration != 0) { static_cast<void>(run_.advance_hardware_reset_ticks(machine_, duration)); }
}

void DspFetchScheduledRun::complete_native(const CycleInputs &inputs,
                                           const Observer &after_cycle,
                                           const unsigned maximum_cycles,
                                           unsigned &cycles) {
  if (run_.hardware_reset_active()) {
    const auto recovery = native_deadline();
    if (!recovery) { throw Fault("DSP pipeline lost its pending reset recovery deadline"); }
    reset_until(*recovery, true);
  } else {
    if (cycles == maximum_cycles) { throw Fault("DSP pipeline cycle budget exhausted"); }
    if (!inputs) { throw Fault("DSP pipeline requires explicit native cycle inputs"); }
    const auto value = inputs();
    run_.validate_boundary(machine_);
    const auto result = run_.advance_cycle(machine_, value);
    ++cycles;
    state_.last_native = absolute(run_.elapsed_ticks());
    if (after_cycle) { after_cycle(result); }
    run_.validate_boundary(machine_);
  }
  state_.last_native = absolute(run_.elapsed_ticks());
}

unsigned DspFetchScheduledRun::advance_to(const DspFetchAdvance advance,
                                          const CycleInputs &inputs,
                                          const Observer &after_cycle,
                                          const DspFetchExternalEvents &external) {
  healthy();
  check_horizon(advance.horizon);
  const Guard guard(*this);
  unsigned cycles = 0;
  unsigned events = 0;
  try {
    bind_external(external);
    auto step = next_step(advance.horizon);
    while (step != Step::none) {
      if (step == Step::external) {
        if (events == external.maximum_events) {
          throw Fault("DSP pipeline event budget exhausted");
        }
        reset_until(*state_.next_external, false);
        complete_external(external);
        ++events;
      } else {
        complete_native(inputs, after_cycle, advance.maximum_cycles, cycles);
        if (state_.external_identity) { bind_external(external); }
      }
      step = next_step(advance.horizon);
    }
    reset_until(advance.horizon.ticks, advance.horizon.native == DspScheduleBoundary::through);
    state_.horizon = advance.horizon;
    state_.advanced = true;
    return cycles;
  } catch (const std::exception &error) {
    throw Fault(
        "DSP pipeline scheduled operation horizon=" + std::to_string(advance.horizon.ticks) +
        " pc=" + hex(machine_.cpu().state().pc, 6) + ": " + error.what());
  }
}

void DspFetchScheduledRun::require_reset_boundary() const {
  healthy();
  if (absolute(run_.elapsed_ticks()) != state_.horizon.ticks ||
      (state_.next_external && (*state_.next_external < state_.horizon.ticks ||
                                (*state_.next_external == state_.horizon.ticks &&
                                 state_.horizon.external != DspScheduleBoundary::before)))) {
    throw Fault("DSP pipeline reset requires an exactly consumed timeline boundary");
  }
}

void DspFetchScheduledRun::begin_hardware_reset(const DspHardwareResetInputs inputs) {
  require_reset_boundary();
  const Guard guard(*this);
  run_.begin_hardware_reset(machine_, inputs);
}

void DspFetchScheduledRun::release_hardware_reset() {
  require_reset_boundary();
  const Guard guard(*this);
  run_.release_hardware_reset(machine_);
}

DspFetchScheduleCheckpoint DspFetchScheduledRun::checkpoint() const {
  healthy();
  return DspFetchScheduleCheckpoint(std::make_shared<const DspFetchScheduleCheckpoint::Storage>(
      DspFetchScheduleCheckpoint::Storage{.inputs = inputs_,
                                          .state = state_,
                                          .native = run_.checkpoint(machine_),
                                          .owner = &machine_,
                                          .lifetime = lifetime_}));
}

void DspFetchScheduledRun::restore(const DspFetchScheduleCheckpoint &checkpoint) {
  if (active_ || !checkpoint.storage_ || checkpoint.storage_->owner != &machine_ ||
      checkpoint.storage_->lifetime.lock() != lifetime_ || checkpoint.storage_->inputs != inputs_) {
    throw Fault("DSP pipeline restore requires its matching inactive owner and contract");
  }
  const Guard guard(*this);
  faulted_ = true;
  run_.restore(checkpoint.storage_->native, machine_);
  state_ = checkpoint.storage_->state;
  faulted_ = false;
}
} // namespace blofeld
