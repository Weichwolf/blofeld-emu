// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
class PeripheralQueryGuard {
public:
  explicit PeripheralQueryGuard(bool &active) : active_(active) {
    if (active_) { throw Fault("Peripheral query cannot reenter its owner"); }
    active_ = true;
  }

  ~PeripheralQueryGuard() { active_ = false; }

  PeripheralQueryGuard(const PeripheralQueryGuard &) = delete;
  PeripheralQueryGuard &operator=(const PeripheralQueryGuard &) = delete;

private:
  bool &active_;
};
}

PeripheralCalendarBoundary
DspFetchScheduledRun::peripheral_boundary(const DspFetchHorizon horizon) {
  return {.ticks = horizon.ticks,
          .native_through = horizon.native == DspScheduleBoundary::through,
          .events_through = horizon.external == DspScheduleBoundary::through};
}

bool DspFetchScheduledRun::peripheral_supplied(const DspFetchPeripheralEvents &peripheral) {
  return peripheral.identity || peripheral.next_ticks || peripheral.observe ||
         peripheral.order != DspExternalOrder::unqualified || peripheral.maximum_events != 0;
}

void DspFetchScheduledRun::check_peripheral(const DspFetchPeripheralEvents &peripheral) {
  if (!peripheral.identity || !peripheral.next_ticks || !peripheral.observe ||
      peripheral.maximum_events == 0 || peripheral.order != DspExternalOrder::external_before_dsp) {
    throw Fault(
        "Peripheral events require identity,callbacks,budget and input-before-native order");
  }
}

std::optional<std::uint64_t>
DspFetchScheduledRun::query_peripheral(const DspFetchPeripheralEvents &peripheral) const {
  const PeripheralQueryGuard guard(querying_peripheral_);
  const auto identity = peripheral.identity;
  const auto budget = peripheral.maximum_events;
  const auto order = peripheral.order;
  const auto next = peripheral.next_ticks();
  run_.validate_boundary(machine_);
  if (peripheral.identity != identity || peripheral.maximum_events != budget ||
      peripheral.order != order) {
    throw Fault("Peripheral query changed its source contract");
  }
  return next;
}

void DspFetchScheduledRun::bind_peripheral(const DspFetchPeripheralEvents &peripheral) {
  if (!peripheral_supplied(peripheral)) {
    if (peripheral_budget_ != 0) { throw Fault("Bound peripheral source was omitted"); }
    return;
  }
  check_peripheral(peripheral);
  if (state_.external_identity && state_.order != DspExternalOrder::external_before_dsp) {
    throw Fault("Peripheral lane requires inherited input before peripheral before native");
  }
  if (peripheral_budget_ == 0) {
    if (state_.advanced) { throw Fault("Peripheral source must bind on its first advance"); }
    peripheral_->reset_unbound(peripheral_boundary(state_.horizon));
    peripheral_budget_ = peripheral.maximum_events;
  }
  if (peripheral.maximum_events != peripheral_budget_) {
    throw Fault("Peripheral source changed its fixed event budget");
  }
  peripheral_->bind(peripheral.identity, query_peripheral(peripheral));
}

void DspFetchScheduledRun::verify_peripheral(const DspFetchPeripheralEvents &peripheral) const {
  if (peripheral_budget_ == 0) {
    if (peripheral_supplied(peripheral)) { throw Fault("Peripheral source is not bound"); }
    return;
  }
  check_peripheral(peripheral);
  if (peripheral.maximum_events != peripheral_budget_) {
    throw Fault("Peripheral source changed its fixed event budget");
  }
  peripheral_->verify(peripheral.identity, query_peripheral(peripheral));
}

void DspFetchScheduledRun::refresh_peripheral(const PeripheralCalendarCause cause,
                                              const std::uint64_t ticks,
                                              const DspFetchPeripheralEvents &peripheral) {
  if (peripheral_budget_ == 0) { return; }
  check_peripheral(peripheral);
  if (peripheral.identity != peripheral_->state().identity ||
      peripheral.maximum_events != peripheral_budget_) {
    throw Fault("Peripheral cause changed its source contract");
  }
  peripheral_->caused(cause, ticks, query_peripheral(peripheral));
}

void DspFetchScheduledRun::complete_peripheral(const DspFetchPeripheralEvents &peripheral) {
  if (peripheral_budget_ == 0) { throw Fault("Peripheral dispatch requires a bound calendar"); }
  verify_peripheral(peripheral);
  const auto ticks = peripheral_->state().pending;
  if (!ticks) { throw Fault("Peripheral dispatch requires a pending deadline"); }
  reset_until(*ticks, false);
  peripheral.observe(*ticks);
  run_.validate_boundary(machine_);
  refresh_peripheral(PeripheralCalendarCause::peripheral, *ticks, peripheral);
}

unsigned DspFetchScheduledRun::event_budget(const DspFetchExternalEvents &external,
                                            const DspFetchPeripheralEvents &peripheral) const {
  if (peripheral_budget_ == 0) { return external.maximum_events; }
  if (state_.external_identity && external.maximum_events != peripheral.maximum_events) {
    throw Fault("Inherited and peripheral events require one equal shared event budget");
  }
  return peripheral.maximum_events;
}

std::optional<PeripheralCalendarState> DspFetchScheduledRun::peripheral_state() const {
  return peripheral_budget_ != 0 ? std::optional{peripheral_->state()} : std::nullopt;
}

}
