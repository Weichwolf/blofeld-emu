// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
bool DspRunClock::hardware_reset_active() const {
  return reset_ && reset_->phase != DspHardwareResetPhase::complete;
}

void DspRunClock::begin_hardware_reset(const std::uint32_t recovery_cycles) {
  if (faulted_ || active_ || hardware_reset_active() || recovery_cycles < 2 ||
      epoch_native_cycle_units() != 0) {
    throw Fault("DSP reset hold requires a fresh native epoch and explicit recovery of at least2 "
                "bypass EXTAL periods");
  }
  reset_ = DspHardwareResetState{.required_recovery_cycles = recovery_cycles};
}

std::uint64_t DspRunClock::advance_hardware_reset(const std::uint64_t maximum_extal_cycles) {
  const auto period = extal_period_ticks();
  if (!reset_ || reset_->phase_ticks != 0 ||
      maximum_extal_cycles > std::numeric_limits<std::uint64_t>::max() / period) {
    throw Fault("Whole-period reset advance requires an aligned representable EXTAL duration");
  }
  return advance_hardware_reset_ticks(maximum_extal_cycles * period) / period;
}

std::optional<std::uint64_t> DspRunClock::hardware_reset_remaining_ticks() const {
  if (!reset_ || reset_->phase != DspHardwareResetPhase::recovering) { return std::nullopt; }
  return ((reset_->required_recovery_cycles - reset_->recovery_extal_cycles) *
          extal_period_ticks()) -
         reset_->phase_ticks;
}

std::uint64_t DspRunClock::advance_hardware_reset_ticks(const std::uint64_t maximum_ticks) {
  if (faulted_ || active_ || !reset_ || reset_->phase == DspHardwareResetPhase::complete ||
      maximum_ticks == 0) {
    throw Fault("DSP reset clock requires an active reset and positive reference ticks");
  }
  auto next = reset_.value();
  const auto period = extal_period_ticks();
  const auto count =
      std::min(maximum_ticks, hardware_reset_remaining_ticks().value_or(maximum_ticks));
  const auto residue = next.phase_ticks + (count % period);
  const auto cycles = (count / period) + (residue / period);
  if (count > std::numeric_limits<std::uint64_t>::max() - ticks_ ||
      count > std::numeric_limits<std::uint64_t>::max() - reset_ticks_ ||
      (next.phase == DspHardwareResetPhase::asserted &&
       cycles > std::numeric_limits<std::uint64_t>::max() - next.asserted_extal_cycles)) {
    throw Fault("DSP hardware reset exhausted its EXTAL/reference-time ledger");
  }
  next.phase_ticks = residue % period;
  if (next.phase == DspHardwareResetPhase::asserted) {
    next.asserted_extal_cycles += cycles;
  } else {
    next.recovery_extal_cycles += cycles;
    if (next.recovery_extal_cycles == next.required_recovery_cycles) {
      next.phase = DspHardwareResetPhase::complete;
    }
  }
  ticks_ += count;
  reset_ticks_ += count;
  reset_ = next;
  return count;
}

void DspRunClock::release_hardware_reset() {
  if (faulted_ || active_ || !reset_ || reset_->phase != DspHardwareResetPhase::asserted ||
      reset_->asserted_extal_cycles < 2) {
    throw Fault("DSP reset release requires at least2 observed bypass EXTAL periods under RESET");
  }
  reset_->phase = DspHardwareResetPhase::recovering;
  reset_->phase_ticks = 0;
}
}
