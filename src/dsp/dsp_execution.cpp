// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_execution.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

namespace blofeld {
DspExecution::DspExecution(DspMachine &machine,
                           const std::uint64_t epoch_ticks,
                           const std::optional<DspResetInputs> initial_reset)
    : machine_(machine), identity_(std::make_shared<const bool>(true)), epoch_ticks_(epoch_ticks) {
  machine_.require_unclocked("timeline");
  if (machine_.clock_inputs_.switch_timing != DspClockSwitch::functional_region_boundary) {
    throw Fault("DSP execution requires a functional device clock");
  }
  if (initial_reset) { start_reset(*initial_reset); }
  machine_.clocked_owner_ = identity_;
  accounted_base_ = machine_.cpu_.state().progress.estimated_cycles;
  static_cast<void>(now());
}

void DspExecution::validate_boundary() const {
  if (faulted_ || machine_.clock().faulted() || machine_.clocked_owner_.owner_before(identity_) ||
      identity_.owner_before(machine_.clocked_owner_)) {
    throw Fault("DSP timeline lost its healthy execution owner");
  }
}

std::uint64_t DspExecution::now() const {
  const auto elapsed = machine_.clock().elapsed_ticks();
  if (elapsed > std::numeric_limits<std::uint64_t>::max() - epoch_ticks_) {
    throw Fault("DSP timeline exhausted common time");
  }
  return epoch_ticks_ + elapsed;
}

void DspExecution::start_reset(const DspResetInputs inputs) {
  machine_.check_hardware_reset(inputs);
  machine_.perform_hardware_reset(inputs);
  accounted_base_ = 0;
  remaining_cycles_ = 0;
  available_cycles_ = 0;
}

void DspExecution::begin_hardware_reset(const DspResetInputs inputs) {
  validate_boundary();
  if (machine_.active_) { throw Fault("DSP reset cannot interrupt an active native instruction"); }
  start_reset(inputs);
}

std::uint64_t DspExecution::advance_reset_to(const std::uint64_t horizon) {
  validate_boundary();
  const auto at = now();
  if (!machine_.clock().hardware_reset_active() || horizon < at) {
    throw Fault("DSP reset advance requires held reset and monotonic device time");
  }
  return machine_.clock_.advance_hardware_reset_ticks(horizon - at);
}

void DspExecution::release_hardware_reset() {
  validate_boundary();
  machine_.check_reset_release();
  machine_.clock_.release_hardware_reset();
}

std::uint64_t DspExecution::next_deadline() const {
  validate_boundary();
  const auto at = now();
  const auto period = machine_.clock().next_step_ticks(machine_.bus(), 1);
  if (period > std::numeric_limits<std::uint64_t>::max() - at) {
    throw Fault("DSP timeline exhausted its next instruction boundary");
  }
  return at + period;
}

unsigned DspExecution::capacity(const std::uint64_t horizon, const DspBoundary boundary) const {
  const auto at = now();
  if (horizon <= at) { return 0; }
  auto available = horizon - at;
  if (boundary == DspBoundary::before) { --available; }
  if (const auto device = machine_.clock().next_native_boundary_ticks(machine_.bus())) {
    available = std::min(available, *device);
  }
  const auto period = machine_.clock().next_step_ticks(machine_.bus(), 1);
  return static_cast<unsigned>(
      std::min<std::uint64_t>(available / period, std::numeric_limits<unsigned>::max()));
}
}
