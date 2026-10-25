// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace blofeld {
void DspScheduledRun::require_reset_boundary(const DspMachine &machine) const {
  if (active_ || faulted_ || state_.inputs_.resets != DspResetObservation::reference_extal_hold) {
    throw Fault("Scheduled reset requires an inactive explicit EXTAL-hold observation contract");
  }
  verify(state_, machine);
  if (state_.horizon_ != state_.inputs_.epoch_ticks + state_.completed_ticks_ ||
      (state_.next_external_ && *state_.next_external_ <= state_.horizon_)) {
    throw Fault("Scheduled reset requires its exact consumed owner boundary; pending instruction "
                "time or external events cannot be discarded");
  }
}

void DspScheduledRun::update_reset_clock(const DspMachine &machine) {
  const auto &clock = machine.run_clock();
  state_.completed_ticks_ = clock.elapsed_ticks();
  state_.completed_cycles_ = clock.native_cycle_units();
  state_.native_epoch_ = clock.native_epoch();
  state_.reset_ = clock.hardware_reset_state();
}

void DspScheduledRun::begin_hardware_reset(DspMachine &machine,
                                           const DspHardwareResetInputs inputs) {
  require_reset_boundary(machine);
  try {
    machine.begin_hardware_reset(inputs);
    state_.pending_.reset();
    update_reset_clock(machine);
  } catch (...) {
    faulted_ = machine.timing_faulted();
    throw;
  }
}

void DspScheduledRun::release_hardware_reset(DspMachine &machine) {
  require_reset_boundary(machine);
  const auto reset = machine.run_clock().hardware_reset_state();
  if (reset && (static_cast<std::uint64_t>(reset->required_recovery_cycles) *
                machine.run_clock().extal_period_ticks()) >
                   std::numeric_limits<std::uint64_t>::max() - state_.horizon_) {
    throw Fault("DSP reset recovery exhausted the declared schedule epoch");
  }
  try {
    machine.release_hardware_reset();
    update_reset_clock(machine);
  } catch (...) {
    faulted_ = machine.timing_faulted();
    throw;
  }
}

void DspScheduledRun::advance_reset_to(DspMachine &machine,
                                       const std::uint64_t horizon,
                                       const DspScheduleBoundary boundary) {
  if (!machine.hardware_reset_active()) { return; }
  const auto target = std::min(horizon, state_.next_external_.value_or(horizon));
  const auto current = state_.inputs_.epoch_ticks + state_.completed_ticks_;
  if (target <= current) { return; }
  auto ticks = target - current;
  const auto remaining = machine.run_clock().hardware_reset_remaining_ticks();
  if (remaining && *remaining == ticks && target == horizon &&
      boundary == DspScheduleBoundary::before) {
    --ticks;
  }
  if (ticks == 0) { return; }
  machine.advance_hardware_reset_ticks(ticks);
  update_reset_clock(machine);
}
}
