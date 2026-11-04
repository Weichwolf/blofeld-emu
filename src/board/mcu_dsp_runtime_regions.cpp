// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
void McuDspRuntime::prepare_native_effect(const CycleInputs &inputs) {
  const auto before = detail::mcu_integrity(mcu_);
  const auto registers = dsp_.cpu().registers();
  static_cast<void>(inputs());
  if (detail::mcu_integrity(mcu_) != before || dsp_.cpu().registers() != registers) {
    throw Fault("Native DSP peripheral preparation changed CPU state");
  }
  timeline_->validate_boundary();
}

void McuDspRuntime::observe_native_effect(const unsigned pc, const Observer &observer) {
  if (!observer) { return; }
  const auto ticks = inputs_.epoch_ticks + dsp_.clock().elapsed_ticks();
  const auto cycle = dsp_.bus().last_dma_cycle();
  const auto before = detail::mcu_integrity(mcu_);
  const auto registers = dsp_.cpu().registers();
  observer({.start_ticks = ticks,
            .completion_ticks = ticks,
            .cycle = cycle.value_or(0),
            .result = {.dma = {},
                       .native = std::nullopt,
                       .native_pc = pc,
                       .issued_address = std::nullopt},
            .phase = DspRuntimePhase::peripheral,
            .cycle_known = cycle.has_value()});
  if (detail::mcu_integrity(mcu_) != before || dsp_.cpu().registers() != registers) {
    throw Fault("Native DSP peripheral observation changed CPU state");
  }
  timeline_->validate_boundary();
}

bool McuDspRuntime::advance_native_region(const std::uint64_t horizon,
                                          const OwnDspBoundary boundary,
                                          const CycleInputs &inputs,
                                          const Observer &observer) {
  if (!inputs || (peripheral_events_.identity && !observer) ||
      period_cycles_ > inputs_.maximum_cycles_per_period) {
    throw Fault("Native DSP region requires its actual input/effect owner and remaining budget");
  }

  struct Context {
    McuDspRuntime &runtime;
    const CycleInputs &inputs;
    const Observer &observer;
  } context{.runtime = *this, .inputs = inputs, .observer = observer};

  const DspPeripheralObserver effects{.context = &context,
                                      .preparing =
                                          [](void *pointer, unsigned) {
                                            const auto &call = *static_cast<Context *>(pointer);
                                            call.runtime.prepare_native_effect(call.inputs);
                                          },
                                      .completed =
                                          [](void *pointer, const unsigned pc) {
                                            const auto &call = *static_cast<Context *>(pointer);
                                            call.runtime.observe_native_effect(pc, call.observer);
                                          }};
  const auto before = detail::mcu_integrity(mcu_);
  const auto registers = dsp_.cpu().registers();
  const auto request = inputs();
  if (detail::mcu_integrity(mcu_) != before || dsp_.cpu().registers() != registers) {
    throw Fault("Native DSP region input changed CPU state");
  }
  const auto cycles = dsp_.clock().native_cycle_units();
  const auto result = timeline_->advance_native_region(
      horizon, boundary, request, inputs_.maximum_cycles_per_period - period_cycles_, effects);
  if (!result) { return false; }
  if (result->execution.fault.kind != dsp56300::FaultKind::none) {
    reject_native(result->execution.fault);
  }
  const auto elapsed = dsp_.clock().native_cycle_units() - cycles;
  if (elapsed == 0 || elapsed > inputs_.maximum_cycles_per_period - period_cycles_) {
    throw Fault("Native DSP region lost its actual finite cycle budget");
  }
  period_cycles_ += static_cast<unsigned>(elapsed);
  state_.last_native = result->completion_ticks;
  if (observer) {
    const auto actual = dsp_.bus().last_dma_cycle();
    const auto after = dsp_.cpu().registers();
    observer({.start_ticks = result->start_ticks,
              .completion_ticks = result->completion_ticks,
              .cycle = actual.value_or(0),
              .result = {},
              .phase = DspRuntimePhase::region,
              .region = result->execution,
              .cycle_known = actual.has_value()});
    if (detail::mcu_integrity(mcu_) != before || dsp_.cpu().registers() != after) {
      throw Fault("Native DSP region observer changed CPU state");
    }
    timeline_->validate_boundary();
  }
  peripheral_cause(PeripheralCalendarCause::native, result->completion_ticks);
  return true;
}

void McuDspRuntime::observe_input_event(const InputEvent &event) {
  if (!event.ticks) { throw Fault("Native DSP input dispatch lost its actual event time"); }
  if (event.peripheral) {
    observe_peripheral(*event.ticks);
  } else {
    observe_external(*event.ticks);
  }
}

bool McuDspRuntime::advance_region_reset(const bool native_through,
                                         const bool external_through,
                                         const InputEvent &event,
                                         const CycleInputs &inputs,
                                         const Observer &observer) {
  const auto native = deadline();
  const bool input_due = event.ticks && (*event.ticks < state_.end_ticks ||
                                         (*event.ticks == state_.end_ticks && external_through));
  const bool native_due =
      native < state_.end_ticks || (native == state_.end_ticks && native_through);
  if (input_due && native_due && event.ticks == native && event.order == DspPeerOrder::reject_tie) {
    throw Fault("Unqualified simultaneous DSP reset and input event");
  }
  if (input_due && (!native_due || *event.ticks < native ||
                    (*event.ticks == native && event.order == DspPeerOrder::peer_first))) {
    advance_reset(*event.ticks, false);
    observe_input_event(event);
    return true;
  }
  if (native_due) {
    advance_native(native, inputs, observer);
    return true;
  }
  advance_reset(state_.end_ticks, native_through);
  return false;
}

void McuDspRuntime::advance_regions(const bool native_through,
                                    const bool external_through,
                                    const CycleInputs &inputs,
                                    const Observer &observer) {
  while (true) {
    const auto event = next_event();
    if (dsp_.clock().hardware_reset_active()) {
      if (advance_region_reset(native_through, external_through, event, inputs, observer)) {
        continue;
      }
      break;
    }
    const bool input_due = event.ticks && (*event.ticks < state_.end_ticks ||
                                           (*event.ticks == state_.end_ticks && external_through));
    const auto horizon = input_due ? *event.ticks : state_.end_ticks;
    const bool through = (horizon != state_.end_ticks || native_through) &&
                         (!input_due || event.order == DspPeerOrder::dsp_first);
    if (advance_native_region(horizon,
                              through ? OwnDspBoundary::through : OwnDspBoundary::before,
                              inputs,
                              observer)) {
      continue;
    }
    if (!input_due) { break; }
    if (event.order == DspPeerOrder::reject_tie && deadline() == *event.ticks) {
      throw Fault("Unqualified simultaneous DSP native completion and input event");
    }
    observe_input_event(event);
  }
  if (external_through) { state_.closed_external = state_.end_ticks; }
  finish_peripheral(native_through, external_through);
}
}
