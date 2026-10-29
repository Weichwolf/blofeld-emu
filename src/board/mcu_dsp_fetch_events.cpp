// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
using detail::mcu_boundary;

DspFetchExternalEvents McuDspFetchSchedule::checked_events() const {
  auto result = external_;
  if (external_.next_ticks) {
    result.next_ticks = [this] {
      const auto before = mcu_boundary(mcu_);
      const auto next = external_.next_ticks();
      if (mcu_boundary(mcu_) != before) {
        throw Fault("MCU/DSP external query changed native MCU state or time");
      }
      if (next && *next < initial_mcu_ticks_) {
        throw Fault("MCU/DSP external event predates its initial native MCU boundary");
      }
      if (next && shared_write_ && shared_write_observed_ && *next == end_ticks_ &&
          inputs_.external_write_order != McuExternalWriteOrder::mcu_before_external) {
        throw Fault("New external event cannot precede its completed MCU shared-write cause");
      }
      return next;
    };
  }
  if (external_.observe) {
    result.observe = [this](const std::uint64_t ticks) {
      const auto before = mcu_boundary(mcu_);
      external_.observe(ticks);
      if (mcu_boundary(mcu_) != before) {
        throw Fault("MCU/DSP external observation changed native MCU state or time");
      }
    };
  }
  return result;
}

void McuDspFetchSchedule::advance_horizon(const DspFetchHorizon horizon,
                                          const CycleInputs &inputs,
                                          const Observer &after_dsp) {
  if (period_cycles_ > inputs_.maximum_dsp_cycles_per_period) {
    throw Fault("MCU/DSP fetch native period counter exceeded its declared budget");
  }
  const auto available = inputs_.maximum_dsp_cycles_per_period - period_cycles_;
  const auto done = dsp_schedule_.advance_to({.horizon = horizon, .maximum_cycles = available},
                                             inputs,
                                             after_dsp,
                                             checked_events(),
                                             checked_peripheral());
  period_cycles_ += done;
}

void McuDspFetchSchedule::prepare_shared_write(const CycleInputs &inputs,
                                               const Observer &after_dsp) {
  const auto address = cycle_.address;
  if (!shared_write_ || !address) {
    throw Fault("MCU/DSP shared-write preparation requires an actual write address");
  }
  const bool native = deadline() == end_ticks_;
  const auto state = dsp_schedule_.state();
  const auto input = next_input_ticks();
  const bool external = input && *input == end_ticks_;
  if (native) {
    if (ties_ == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("MCU/DSP fetch shared write tie counter exhausted");
    }
    ++ties_;
    if (inputs_.shared_write_order == McuDspTieOrder::unqualified) {
      throw Fault("Unqualified simultaneous DSP pipeline cycle and MCU shared write at " +
                  hex(*address) + " mcu-pc=" + hex(cycle_.context.pc));
    }
  }
  if (external && inputs_.external_write_order == McuExternalWriteOrder::unqualified) {
    throw Fault("Unqualified simultaneous external event and MCU shared write at " + hex(*address) +
                " mcu-pc=" + hex(cycle_.context.pc));
  }
  const bool native_first = inputs_.shared_write_order == McuDspTieOrder::dsp_before_mcu;
  const bool external_first =
      inputs_.external_write_order == McuExternalWriteOrder::external_before_mcu;
  if (native && external) {
    if (state.order == DspExternalOrder::reject_tie) {
      throw Fault("Unqualified simultaneous external event and DSP pipeline at MCU shared write");
    }
    const bool event_before_dsp =
        peripheral_.identity || state.order == DspExternalOrder::external_before_dsp;
    if (native_first != external_first && event_before_dsp != external_first) {
      throw Fault("Cyclic MCU/DSP/external shared-write tie contract");
    }
  }
  advance_horizon(
      {.ticks = end_ticks_,
       .native = native_first ? DspScheduleBoundary::through : DspScheduleBoundary::before,
       .external = external_first ? DspScheduleBoundary::through : DspScheduleBoundary::before},
      inputs,
      after_dsp);
  const auto pending = next_input_ticks();
  if (pending && *pending == end_ticks_ &&
      inputs_.external_write_order == McuExternalWriteOrder::unqualified) {
    throw Fault("Unqualified newborn external event at MCU shared write");
  }
}

void McuDspFetchSchedule::verify_external_boundary() const {
  const auto state = dsp_schedule_.state();
  const auto pending = next_input_ticks();
  if (state.horizon.ticks != end_ticks_ || state.horizon.native != DspScheduleBoundary::through ||
      state.horizon.external != DspScheduleBoundary::through ||
      (pending && *pending <= end_ticks_)) {
    throw Fault("MCU/DSP fetch completed boundary lost external or native timeline work");
  }
}
} // namespace blofeld
