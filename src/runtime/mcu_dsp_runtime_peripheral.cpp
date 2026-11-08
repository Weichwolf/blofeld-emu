// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_execution.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "runtime/mcu_dsp_runtime.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
void McuDspRuntime::check_peripheral_contract() const {
  const bool present = peripheral_events_.identity != nullptr;
  if (present != static_cast<bool>(peripheral_events_.next_ticks) ||
      present != static_cast<bool>(peripheral_events_.observe) ||
      (present && peripheral_events_.order != DspPeerOrder::peer_first) ||
      (!present && peripheral_events_.order != DspPeerOrder::unqualified)) {
    throw Fault("MCU/DSP peripheral source requires coherent identity, callbacks and order");
  }
  if (present && external_.identity && external_.order != DspPeerOrder::peer_first) {
    throw Fault("MCU/DSP combined input sources require inherited/peripheral/DSP order");
  }
}

std::optional<std::uint64_t> McuDspRuntime::query_peripheral() const {
  if (!peripheral_events_.identity) { return std::nullopt; }
  const auto result = peripheral_events_.next_ticks();
  timeline_->validate_boundary();
  return result;
}

std::optional<std::uint64_t> McuDspRuntime::next_peripheral() const {
  if (!peripheral_) { return std::nullopt; }
  const auto observed = query_peripheral();
  peripheral_->verify(peripheral_events_.identity, observed);
  return observed;
}

std::optional<std::uint64_t> McuDspRuntime::next_input() {
  return next_event().ticks;
}

McuDspRuntime::InputEvent McuDspRuntime::next_event() {
  const auto inherited = next_external();
  const auto peripheral = next_peripheral();
  if (peripheral && (!inherited || *peripheral < *inherited)) {
    return {.ticks = peripheral, .order = peripheral_events_.order, .peripheral = true};
  }
  return {.ticks = inherited, .order = external_.order, .peripheral = false};
}

DspPeerOrder McuDspRuntime::input_order(const std::uint64_t ticks) const {
  if (state_.pending_external == ticks) { return external_.order; }
  return peripheral_events_.order;
}

void McuDspRuntime::peripheral_cause(const PeripheralCalendarCause cause,
                                     const std::uint64_t ticks) {
  if (peripheral_) { peripheral_->caused(cause, ticks, query_peripheral()); }
}

void McuDspRuntime::finish_peripheral(const bool native_through, const bool events_through) {
  if (peripheral_) {
    peripheral_->finish({.ticks = state_.end_ticks,
                         .native_through = native_through,
                         .events_through = events_through});
  }
}

void McuDspRuntime::observe_peripheral(const std::uint64_t ticks) {
  peripheral_events_.observe(ticks);
  timeline_->validate_boundary();
  peripheral_cause(PeripheralCalendarCause::peripheral, ticks);
}

}
