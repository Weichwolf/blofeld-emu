// SPDX-License-Identifier: GPL-3.0-only
#include "nand_native_run.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "nand_native_peer.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace blofeld::test {
namespace {
constexpr std::uint64_t power_tick = 108000;
constexpr unsigned recovery_periods = 2, event_budget = 4;
constexpr auto timing =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
constexpr OwnDspResetInputs reset{
    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = recovery_periods};
}

NandNativeCpu native_cpu(const OwnDspMachine &machine) {
  return {.cpu = machine.cpu().registers(), .progress = machine.cpu().progress()};
}

NandNativeRun::NandNativeRun(OwnDspMachine &machine, NandPeer &peer, NandPeerEvents inherited)
    : machine_(machine),
      peer_(peer),
      inherited_(std::move(inherited)),
      state_{.external_identity = inherited_.identity},
      timeline_(std::make_unique<OwnDspTimeline>(machine, timing, 0, reset)) {
  if (!inherited_.identity || !inherited_.next_ticks || !inherited_.observe) {
    throw Fault("NAND native fixture requires its actual inherited input source");
  }
  state_.next_external = inherited_.next_ticks();
  const auto events = peer_.events();
  calendar_.bind(events.identity, events.next_ticks());
  reset_to(power_tick);
  timeline_->release_hardware_reset();
  reset_to(power_tick + (recovery_periods * machine_.clock().extal_period_ticks()));
}

void NandNativeRun::verify() const {
  if (faulted_ || state_.horizon != machine_.clock().elapsed_ticks() ||
      state_.external_identity != inherited_.identity ||
      state_.next_external != inherited_.next_ticks()) {
    throw Fault("NAND native fixture lost its healthy clock/input boundary");
  }
  const auto events = peer_.events();
  calendar_.verify(events.identity, events.next_ticks());
}

void NandNativeRun::drain(const std::uint64_t horizon, const std::optional<std::uint64_t> native) {
  const auto events = peer_.events();
  for (unsigned budget = 0;; ++budget) {
    const auto peripheral = events.next_ticks();
    const auto external = state_.next_external;
    const bool input_first = external && *external <= horizon &&
                             (!peripheral || *external <= *peripheral) &&
                             (!native || *external <= *native);
    const bool nand_first = calendar_.first(native, external, {.ticks = horizon});
    if (!input_first && !nand_first) { return; }
    if (budget == event_budget) { throw Fault("NAND fixture exceeded its explicit event budget"); }
    if (input_first) {
      if (*external < state_.horizon ||
          state_.external_events == std::numeric_limits<std::uint64_t>::max()) {
        throw Fault("NAND inherited input moved backwards or exhausted its counter");
      }
      inherited_.observe(*external);
      state_.next_external = inherited_.next_ticks();
      if (state_.next_external && *state_.next_external <= *external) {
        throw Fault("NAND inherited input did not consume its exact event");
      }
      ++state_.external_events;
      calendar_.caused(PeripheralCalendarCause::external, *external, events.next_ticks());
    } else {
      if (!peripheral) { throw Fault("NAND calendar offered an absent peripheral deadline"); }
      events.observe(*peripheral);
      calendar_.caused(PeripheralCalendarCause::peripheral, *peripheral, events.next_ticks());
    }
  }
}

void NandNativeRun::reset_to(const std::uint64_t horizon) {
  verify();
  drain(horizon, std::nullopt);
  const auto before = state_.horizon;
  if (timeline_->advance_reset_to(horizon) != horizon - before) {
    throw Fault("NAND fixture lost an actual held-reset/recovery period");
  }
  peer_.before_native(horizon);
  state_.horizon = horizon;
  calendar_.finish({.ticks = horizon});
}

OwnDspFetchCycleResult NandNativeRun::tick() {
  verify();
  faulted_ = true;
  const auto deadline = timeline_->next_deadline();
  drain(deadline, deadline);
  peer_.before_native(machine_.clock().elapsed_ticks());
  const auto previous = machine_.bus().last_dma_cycle();
  if (previous == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("NAND fixture DMA counter exhausted");
  }
  const auto cycle = timeline_->advance_one(deadline,
                                            OwnDspBoundary::through,
                                            {.cycle = previous ? *previous + 1 : 0,
                                             .known_requests = 15,
                                             .timing = DspDmaTiming::reference_sampled_cycles,
                                             .grant = DspDmaGrant::available});
  if (!cycle) { throw Fault("NAND fixture missed its actual native reference deadline"); }
  if (cycle->native && cycle->native->fault.kind != dsp56300::FaultKind::none) {
    const auto &fault = cycle->native->fault;
    throw Fault("NAND own-core operation=" + hex(static_cast<unsigned>(fault.kind), 2) +
                " space=" + hex(static_cast<unsigned>(fault.space), 2) +
                " address=" + hex(fault.address, 6) + " value=" + hex(fault.value, 6) +
                " pc=" + hex(fault.pc, 6) + " ticks=" + std::to_string(deadline));
  }
  peer_.after_native(deadline, cycle->native_pc);
  calendar_.caused(PeripheralCalendarCause::native, deadline, peer_.events().next_ticks());
  calendar_.finish({.ticks = deadline});
  state_.horizon = deadline;
  faulted_ = false;
  return *cycle;
}

PeripheralCalendarState NandNativeRun::peripheral_state() const {
  verify();
  return calendar_.state();
}

NandNativeState NandNativeRun::state() const {
  verify();
  return state_;
}
}
