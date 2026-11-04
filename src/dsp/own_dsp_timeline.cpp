// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_timeline.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace blofeld {
OwnDspTimeline::OwnDspTimeline(OwnDspMachine &machine,
                               const OwnDspAutomaticTiming timing,
                               const std::uint64_t epoch_ticks,
                               const std::optional<OwnDspResetInputs> initial_reset)
    : machine_(machine),
      timing_(timing),
      epoch_ticks_(epoch_ticks),
      expected_ticks_(machine.clock().elapsed_ticks()),
      expected_epoch_(machine.clock().native_epoch()),
      expected_dma_(machine.bus().last_dma_cycle()),
      horizon_(epoch_ticks),
      native_history_(machine.cpu().progress()) {
  const auto clock_mode = machine_.clock_inputs_.switch_timing;
  const bool native = native_regions() && clock_mode == DspClockSwitch::functional_region_boundary;
  const bool diagnostic =
      clock_mode == DspClockSwitch::reference_cycle_boundary &&
      (timing_ == OwnDspAutomaticTiming::one_effect_per_cycle ||
       timing_ == OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority ||
       timing_ == OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts ||
       timing_ == OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts ||
       timing_ == OwnDspAutomaticTiming::native_cycle_budget);
  if ((!native && !diagnostic) ||
      expected_ticks_ > std::numeric_limits<std::uint64_t>::max() - epoch_ticks_) {
    throw Fault(
        "Own DSP timeline requires a qualified clock/frontend pair and representable epoch");
  }
  cycle_identity_ = std::make_shared<const bool>(true);
  if (initial_reset) {
    if (!machine_.captured_fetch_owner_.expired()) {
      throw Fault("Own DSP timeline reset requires an unowned machine");
    }
    start_reset(*initial_reset);
  } else if (native_regions()) {
    machine_.require_running();
  } else {
    fetch_ = std::make_unique<OwnDspFetch>(machine_);
  }
  machine_.clocked_owner_ = cycle_identity_;
  horizon_ = epoch_ticks_ + expected_ticks_;
  if (fetch_) { static_cast<void>(next_deadline()); }
}

void OwnDspTimeline::check_owner() const {
  if (machine_.clocked_owner_.owner_before(cycle_identity_) ||
      cycle_identity_.owner_before(machine_.clocked_owner_) || faulted_ ||
      machine_.clock().faulted() || machine_.clock().elapsed_ticks() != expected_ticks_ ||
      machine_.clock().native_epoch() != expected_epoch_ ||
      machine_.bus().last_dma_cycle() != expected_dma_) {
    throw Fault("Own DSP timeline requires its healthy unchanged native boundary");
  }
  if (native_regions() && !reset_identity_) {
    if (!machine_.captured_fetch_owner_.expired()) {
      throw Fault("Native DSP timeline lost its unshared instruction owner");
    }
    return;
  }
  if (fetch_) {
    static_cast<void>(fetch_->view());
  } else if (!reset_identity_ || machine_.captured_fetch_owner_.owner_before(reset_identity_) ||
             reset_identity_.owner_before(machine_.captured_fetch_owner_)) {
    throw Fault("Own DSP timeline reset ownership is stale");
  }
}

void OwnDspTimeline::start_reset(const OwnDspResetInputs inputs) {
  machine_.check_hardware_reset(inputs);
  const auto identity = std::make_shared<const bool>(true);
  try {
    machine_.perform_hardware_reset(inputs);
    native_words_.reset();
    native_cost_ = 0;
    native_remaining_ = 0;
    native_dependencies_ = 0;
    native_dependency_remaining_ = 0;
    native_history_.reset(machine_.cpu_.progress());
    native_staged_ = false;
    fetch_.reset();
    reset_identity_ = identity;
    machine_.captured_fetch_owner_ = reset_identity_;
    machine_.clocked_owner_ = cycle_identity_;
    expected_epoch_ = machine_.clock().native_epoch();
    expected_dma_ = machine_.bus().last_dma_cycle();
    horizon_ = epoch_ticks_ + expected_ticks_;
    boundary_ = OwnDspBoundary::through;
  } catch (...) {
    faulted_ = true;
    throw;
  }
}

void OwnDspTimeline::begin_hardware_reset(const OwnDspResetInputs inputs) {
  check_owner();
  if ((!fetch_ && !native_regions()) || horizon_ != epoch_ticks_ + expected_ticks_) {
    throw Fault("Own DSP timeline reset requires its current completed reference boundary");
  }
  start_reset(inputs);
}

std::uint64_t OwnDspTimeline::advance_reset_to(const std::uint64_t horizon) {
  check_owner();
  if (!machine_.clock().hardware_reset_active() || horizon < horizon_) {
    throw Fault("Own DSP timeline reset advance requires held reset and monotonic time");
  }
  const auto now = epoch_ticks_ + expected_ticks_;
  if (horizon == now) { return 0; }
  try {
    const auto consumed = machine_.clock_.advance_hardware_reset_ticks(horizon - now);
    expected_ticks_ = machine_.clock().elapsed_ticks();
    horizon_ = epoch_ticks_ + expected_ticks_;
    boundary_ = OwnDspBoundary::through;
    if (!machine_.clock().hardware_reset_active()) {
      machine_.captured_fetch_owner_.reset();
      reset_identity_.reset();
      if (!native_regions()) { fetch_ = std::make_unique<OwnDspFetch>(machine_); }
    }
    return consumed;
  } catch (...) {
    faulted_ = true;
    throw;
  }
}

void OwnDspTimeline::release_hardware_reset() {
  check_owner();
  if (!machine_.clock().hardware_reset_active()) {
    throw Fault("Own DSP timeline reset is already complete");
  }
  machine_.check_reset_release();
  machine_.clock_.release_hardware_reset();
}

std::uint64_t OwnDspTimeline::next_deadline() const {
  check_owner();
  if (machine_.clock().hardware_reset_active()) {
    throw Fault("Own DSP timeline execution awaits completed reset recovery");
  }
  return cycle_deadline();
}

std::uint64_t OwnDspTimeline::cycle_deadline() const {
  const auto duration = machine_.clock().next_step_ticks(machine_.bus(), 1);
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (duration > maximum - expected_ticks_ || expected_ticks_ + duration > maximum - epoch_ticks_) {
    throw Fault("Own DSP timeline exhausted common time");
  }
  return epoch_ticks_ + expected_ticks_ + duration;
}

std::optional<OwnDspFetchCycleResult> OwnDspTimeline::advance_one(const std::uint64_t horizon,
                                                                  const OwnDspBoundary boundary,
                                                                  const DspDmaInputs inputs) {
  check_owner();
  if (!fetch_ || native_regions()) {
    throw Fault("DSP cycle replay requires its diagnostic fetch owner");
  }
  if ((boundary != OwnDspBoundary::before && boundary != OwnDspBoundary::through) ||
      horizon < horizon_ ||
      (horizon == horizon_ && boundary_ == OwnDspBoundary::through &&
       boundary == OwnDspBoundary::before)) {
    throw Fault("Own DSP timeline cannot rewind a consumed boundary");
  }
  auto attempted_ticks = horizon;
  try {
    attempted_ticks = cycle_deadline();
    if (attempted_ticks > horizon ||
        (attempted_ticks == horizon && boundary == OwnDspBoundary::before)) {
      horizon_ = horizon;
      boundary_ = boundary;
      return std::nullopt;
    }
    auto result = fetch_->advance_dma_automatic(inputs, timing_);
    expected_ticks_ = machine_.clock().elapsed_ticks();
    expected_dma_ = machine_.bus().last_dma_cycle();
    horizon_ = attempted_ticks;
    boundary_ = OwnDspBoundary::through;
    if (result.native && result.native->fault.kind != dsp56300::FaultKind::none) {
      faulted_ = true;
    }
    return result;
  } catch (const Fault &error) {
    faulted_ = true;
    const auto regs = machine_.cpu().registers();
    throw Fault("Own DSP timeline operation=reference-cycle pc=" + hex(regs.pc, 6) +
                " omr=" + hex(regs.omr, 6) + " attempted-ticks=" + std::to_string(attempted_ticks) +
                ": " + error.what());
  } catch (...) {
    faulted_ = true;
    throw;
  }
}
}
