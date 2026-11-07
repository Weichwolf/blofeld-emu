// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_execution.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
void McuDspRuntime::prepare_native_effect(const CycleInputs &inputs) {
  static_cast<void>(inputs());
  timeline_->validate_boundary();
}

void McuDspRuntime::observe_native_effect(const unsigned pc, const Observer &observer) {
  if (!observer) { return; }
  const auto ticks = inputs_.epoch_ticks + dsp_.clock().elapsed_ticks();
  const auto cycle = dsp_.bus().last_dma_cycle();
  observer({.start_ticks = ticks,
            .completion_ticks = ticks,
            .cycle = cycle.value_or(0),
            .native_pc = pc,
            .phase = DspRuntimePhase::peripheral,
            .cycle_known = cycle.has_value()});
  timeline_->validate_boundary();
}

bool McuDspRuntime::advance_native_region(const std::uint64_t horizon,
                                          const DspBoundary boundary,
                                          const CycleInputs &inputs,
                                          const Observer &observer) {
  if (!inputs || (peripheral_events_.identity && !observer)) {
    throw Fault("Native DSP region requires its input and peripheral event owners");
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
  const auto request = inputs();
  const auto cycles = dsp_.clock().elapsed_cycle_units();
  const auto result = timeline_->advance_native_region(horizon, boundary, request, effects);
  if (!result) { return false; }
  if (result->execution && result->execution->fault.kind != dsp56300::FaultKind::none) {
    reject_native(result->execution->fault);
  }
  const auto elapsed = dsp_.clock().elapsed_cycle_units() - cycles;
  if (elapsed == 0) { throw Fault("Native DSP region failed to advance device time"); }
  state_.last_native = result->completion_ticks;
  if (observer) {
    const auto actual = dsp_.bus().last_dma_cycle();
    const auto pc = dsp_.cpu().instruction_fetch().address;
    observer({.start_ticks = result->start_ticks,
              .completion_ticks = result->completion_ticks,
              .cycle = actual.value_or(0),
              .native_pc = pc,
              .phase = DspRuntimePhase::region,
              .region = result->execution,
              .cycle_known = actual.has_value()});
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
                                         const InputEvent &event) {
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
    advance_reset(native, true);
    state_.last_native = native;
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
      if (advance_region_reset(native_through, external_through, event)) { continue; }
      break;
    }
    const bool input_due = event.ticks && (*event.ticks < state_.end_ticks ||
                                           (*event.ticks == state_.end_ticks && external_through));
    const auto horizon = input_due ? *event.ticks : state_.end_ticks;
    const bool through = (horizon != state_.end_ticks || native_through) &&
                         (!input_due || event.order == DspPeerOrder::dsp_first);
    if (advance_native_region(
            horizon, through ? DspBoundary::through : DspBoundary::before, inputs, observer)) {
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
