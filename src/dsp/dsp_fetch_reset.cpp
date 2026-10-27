// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_fetch_run.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
namespace {
class ResetGuard {
public:
  explicit ResetGuard(bool &active) : active_(active) { active_ = true; }

  ~ResetGuard() { active_ = false; }

  ResetGuard(const ResetGuard &) = delete;
  ResetGuard &operator=(const ResetGuard &) = delete;

private:
  bool &active_;
};

void start_fetch(DspMachine &machine, const DspReferenceExecution execution) {
  static_cast<void>(machine.begin_reference_instruction(execution));
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("DSP reset recovery lost its native boot opcode"); }
  static_cast<void>(machine.start_reference_fetch(pending->ticket));
}
}

bool DspFetchRun::hardware_reset_active() const {
  const auto &clock = impl_->state.reset_clock;
  return clock && clock->hardware_reset_active();
}

std::optional<DspHardwareResetState> DspFetchRun::hardware_reset_state() const {
  const auto &clock = impl_->state.reset_clock;
  return clock ? clock->hardware_reset_state() : std::nullopt;
}

std::optional<std::uint64_t> DspFetchRun::hardware_reset_remaining_ticks() const {
  const auto &clock = impl_->state.reset_clock;
  return clock ? clock->hardware_reset_remaining_ticks() : std::nullopt;
}

std::uint64_t DspFetchRun::hardware_reset_ticks() const {
  return impl_->state.reset_ticks;
}

std::uint64_t DspFetchRun::native_epoch() const {
  return impl_->state.epoch;
}

void DspFetchRun::begin_hardware_reset(DspMachine &machine, const DspHardwareResetInputs inputs) {
  check_owner(machine);
  auto &state = impl_->state;
  state.verify(machine);
  constexpr unsigned all_modes = 15;
  constexpr unsigned spi_mode = 5;
  if (state.inputs.resets != DspFetchResetTiming::reference_cycle_boundary ||
      inputs.mode_known != all_modes || inputs.mode_levels != spi_mode || !inputs.pinit ||
      inputs.pinit.value() || inputs.recovery_cycles < 2 || hardware_reset_active() ||
      state.epoch == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP fetch reset requires an inactive explicit cycle-boundary contract, "
                "stable observed mode5/PINIT-low and at least2 recovery EXTAL periods");
  }
  DspRunClock clock({.extal_hz = state.inputs.extal_hz,
                     .timebase_hz = state.inputs.timebase_hz,
                     .switch_timing = DspClockSwitch::reference_instruction_boundary});
  clock.begin_hardware_reset(inputs.recovery_cycles);
  const ResetGuard guard(impl_->active);
  try {
    machine.bus().observe_irqa((inputs.mode_levels & 1U) != 0);
    machine.bus().pll().observe_pinit(inputs.pinit);
    machine.cpu().reset_spi_boot();
    if (state.pipeline) {
      state.pipeline = DspInstructionPipeline{.profile = state.inputs.pipeline,
                                              .view = {.decode_address = machine.cpu().state().pc}};
    }
    state.reset_clock = clock;
    state.epoch_cycles = state.cycles;
    ++state.epoch;
    machine.set_reference_reset(true);
    state.bind(machine);
  } catch (...) {
    fail(machine);
    throw;
  }
}

void DspFetchRun::release_hardware_reset(DspMachine &machine) {
  check_owner(machine);
  auto &state = impl_->state;
  state.verify(machine);
  if (!state.reset_clock || !hardware_reset_active()) {
    throw Fault("DSP fetch reset release requires its active native reset owner");
  }
  const auto reset = state.reset_clock->hardware_reset_state();
  if (!reset) { throw Fault("DSP fetch reset owner lost its EXTAL state"); }
  const auto recovery = static_cast<std::uint64_t>(reset->required_recovery_cycles) *
                        state.reset_clock->extal_period_ticks();
  if (recovery > std::numeric_limits<std::uint64_t>::max() - state.ticks ||
      recovery > std::numeric_limits<std::uint64_t>::max() - state.reset_ticks) {
    throw Fault("DSP fetch reset recovery exhausted its lifetime time ledger");
  }
  // Invalid/too-early release consumes no native state or clock period.
  state.reset_clock->release_hardware_reset();
}

std::uint64_t DspFetchRun::advance_hardware_reset_ticks(DspMachine &machine,
                                                        const std::uint64_t maximum_ticks) {
  check_owner(machine);
  auto &state = impl_->state;
  state.verify(machine);
  if (!state.reset_clock || !hardware_reset_active() || maximum_ticks == 0) {
    throw Fault("DSP fetch reset advance requires an active owner and positive EXTAL ticks");
  }
  auto next = *state.reset_clock;
  const auto ticks = next.advance_hardware_reset_ticks(maximum_ticks);
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (ticks > maximum - state.ticks || ticks > maximum - state.reset_ticks) {
    throw Fault("DSP fetch reset exhausted its lifetime time ledger");
  }
  const ResetGuard guard(impl_->active);
  try {
    state.reset_clock = next;
    state.ticks += ticks;
    state.reset_ticks += ticks;
    if (!hardware_reset_active()) {
      machine.set_reference_reset(false);
      start_fetch(machine, state.inputs.execution);
    }
    state.bind(machine);
    return ticks;
  } catch (...) {
    fail(machine);
    throw;
  }
}
}
