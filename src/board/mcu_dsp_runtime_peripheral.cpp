// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
void McuDspRuntime::check_peripheral_contract() const {
  const bool present = peripheral_events_.identity != nullptr;
  if (present != static_cast<bool>(peripheral_events_.next_ticks) ||
      present != static_cast<bool>(peripheral_events_.observe) ||
      (present && (peripheral_events_.order != DspPeerOrder::peer_first ||
                   peripheral_events_.maximum_events_per_period == 0)) ||
      (!present && (peripheral_events_.order != DspPeerOrder::unqualified ||
                    peripheral_events_.maximum_events_per_period != 0))) {
    throw Fault(
        "MCU/DSP peripheral source requires coherent identity, callbacks, order and budget");
  }
  if (present && external_.identity &&
      (external_.order != DspPeerOrder::peer_first ||
       external_.maximum_events_per_period != peripheral_events_.maximum_events_per_period)) {
    throw Fault(
        "MCU/DSP combined input sources require inherited/peripheral/DSP order and equal budgets");
  }
}

std::optional<std::uint64_t> McuDspRuntime::query_peripheral() const {
  if (!peripheral_events_.identity) { return std::nullopt; }
  const auto before = detail::mcu_integrity(mcu_);
  const auto registers = dsp_.cpu().registers();
  const auto elapsed = dsp_.clock().elapsed_ticks();
  const auto result = peripheral_events_.next_ticks();
  if (detail::mcu_integrity(mcu_) != before || dsp_.cpu().registers() != registers ||
      dsp_.clock().elapsed_ticks() != elapsed) {
    throw Fault("MCU/DSP peripheral query changed CPU state or time");
  }
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

void McuDspRuntime::count_input_event() {
  const auto budget = external_.identity ? external_.maximum_events_per_period
                                         : peripheral_events_.maximum_events_per_period;
  if (period_events_ >= budget) {
    throw Fault(peripheral_ ? "MCU/DSP input sources exceeded their shared period budget"
                            : "MCU/DSP external source exceeded its period budget");
  }
  ++period_events_;
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
  count_input_event();
  const auto before = detail::mcu_integrity(mcu_);
  const auto registers = dsp_.cpu().registers();
  const auto elapsed = dsp_.clock().elapsed_ticks();
  peripheral_events_.observe(ticks);
  if (detail::mcu_integrity(mcu_) != before || dsp_.cpu().registers() != registers ||
      dsp_.clock().elapsed_ticks() != elapsed) {
    throw Fault("MCU/DSP peripheral observation changed CPU state or time");
  }
  timeline_->validate_boundary();
  peripheral_cause(PeripheralCalendarCause::peripheral, ticks);
}

std::optional<PeripheralCalendarState> McuDspRuntime::peripheral_state() const {
  return peripheral_ ? std::optional{peripheral_->state()} : std::nullopt;
}
}
