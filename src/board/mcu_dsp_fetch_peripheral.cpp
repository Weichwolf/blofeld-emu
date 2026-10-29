// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
DspFetchPeripheralEvents
McuDspFetchSchedule::guarded_peripheral(const DspFetchPeripheralEvents &source,
                                        const McuMachine &mcu) {
  auto result = source;
  if (source.next_ticks) {
    result.next_ticks = [&mcu, query = source.next_ticks] {
      const auto before = detail::mcu_boundary(mcu);
      const auto next = query();
      if (detail::mcu_boundary(mcu) != before) {
        throw Fault("Peripheral query changed native MCU state or time");
      }
      return next;
    };
  }
  if (source.observe) {
    result.observe = [&mcu, observe = source.observe](const std::uint64_t ticks) {
      const auto before = detail::mcu_boundary(mcu);
      observe(ticks);
      if (detail::mcu_boundary(mcu) != before) {
        throw Fault("Peripheral observation changed native MCU state or time");
      }
    };
  }
  return result;
}

DspFetchPeripheralEvents McuDspFetchSchedule::checked_peripheral() const {
  auto result = peripheral_;
  if (peripheral_.next_ticks) {
    result.next_ticks = [this] {
      const auto before = detail::mcu_boundary(mcu_);
      const auto next = peripheral_.next_ticks();
      if (detail::mcu_boundary(mcu_) != before) {
        throw Fault("Peripheral query changed native MCU state or time");
      }
      if (next && *next < initial_mcu_ticks_) {
        throw Fault("Peripheral event predates its initial native MCU boundary");
      }
      if (next && shared_write_ && shared_write_observed_ && *next == end_ticks_ &&
          inputs_.external_write_order != McuExternalWriteOrder::mcu_before_external) {
        throw Fault("New peripheral event cannot precede its completed MCU shared-write cause");
      }
      return next;
    };
  }
  if (peripheral_.observe) {
    result.observe = [this](const std::uint64_t ticks) {
      const auto before = detail::mcu_boundary(mcu_);
      peripheral_.observe(ticks);
      if (detail::mcu_boundary(mcu_) != before) {
        throw Fault("Peripheral observation changed native MCU state or time");
      }
    };
  }
  return result;
}

std::optional<std::uint64_t> McuDspFetchSchedule::next_input_ticks() const {
  const auto inherited = dsp_schedule_.state().next_external;
  const auto state = dsp_schedule_.peripheral_state();
  const auto pending = state ? state->pending : std::nullopt;
  if (!pending) { return inherited; }
  if (!inherited || *pending < *inherited) { return pending; }
  return inherited;
}
}
