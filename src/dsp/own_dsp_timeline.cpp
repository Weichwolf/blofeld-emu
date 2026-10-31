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
#include <optional>

namespace blofeld {
OwnDspTimeline::OwnDspTimeline(OwnDspMachine &machine,
                               const OwnDspAutomaticTiming timing,
                               const std::uint64_t epoch_ticks)
    : machine_(machine),
      fetch_(machine),
      timing_(timing),
      epoch_ticks_(epoch_ticks),
      expected_ticks_(machine.clock().elapsed_ticks()),
      expected_epoch_(machine.clock().native_epoch()),
      expected_dma_(machine.bus().last_dma_cycle()),
      horizon_(epoch_ticks) {
  if ((timing_ != OwnDspAutomaticTiming::one_effect_per_cycle &&
       timing_ != OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority &&
       timing_ != OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts) ||
      expected_ticks_ > std::numeric_limits<std::uint64_t>::max() - epoch_ticks_) {
    throw Fault("Own DSP timeline requires qualified timing and representable epoch");
  }
  horizon_ += expected_ticks_;
  static_cast<void>(next_deadline());
}

void OwnDspTimeline::check_owner() const {
  if (faulted_ || machine_.clock().faulted() ||
      machine_.clock().elapsed_ticks() != expected_ticks_ ||
      machine_.clock().native_epoch() != expected_epoch_ ||
      machine_.bus().last_dma_cycle() != expected_dma_) {
    throw Fault("Own DSP timeline requires its healthy unchanged native boundary");
  }
  static_cast<void>(fetch_.view());
}

std::uint64_t OwnDspTimeline::next_deadline() const {
  check_owner();
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
  if ((boundary != OwnDspBoundary::before && boundary != OwnDspBoundary::through) ||
      horizon < horizon_ ||
      (horizon == horizon_ && boundary_ == OwnDspBoundary::through &&
       boundary == OwnDspBoundary::before)) {
    throw Fault("Own DSP timeline cannot rewind a consumed boundary");
  }
  const auto deadline = next_deadline();
  if (deadline > horizon || (deadline == horizon && boundary == OwnDspBoundary::before)) {
    horizon_ = horizon;
    boundary_ = boundary;
    return std::nullopt;
  }
  try {
    auto result = fetch_.advance_dma_automatic(inputs, timing_);
    expected_ticks_ = machine_.clock().elapsed_ticks();
    expected_dma_ = machine_.bus().last_dma_cycle();
    horizon_ = deadline;
    boundary_ = OwnDspBoundary::through;
    if (result.native && result.native->fault.kind != dsp56300::FaultKind::none) {
      faulted_ = true;
    }
    return result;
  } catch (...) {
    faulted_ = true;
    throw;
  }
}
}
