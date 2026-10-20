// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_scheduled_run.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
std::optional<std::uint64_t>
DspScheduledRun::query_external(DspMachine &machine, const DspExternalEvents &external) const {
  const auto query = external.next_ticks;
  const auto expected = machine.cpu().debug_state();
  const auto next = query();
  verify(state_, machine);
  if (machine.cpu().debug_state() != expected) {
    throw Fault("External event query changed the native CPU state");
  }
  if (next && ((state_.last_external_ && *next <= *state_.last_external_) ||
               (!state_.last_external_ && *next < state_.horizon_))) {
    throw Fault("External event timestamps must increase from the initial schedule horizon");
  }
  return next;
}

void DspScheduledRun::bind_external(DspMachine &machine, const DspExternalEvents &external) {
  const bool supplied = external.next_ticks || external.observe;
  if (!supplied) {
    if (state_.external_bound_) { throw Fault("Bound external event source was omitted"); }
    if (external.order != DspExternalOrder::unqualified || external.maximum_events != 0) {
      throw Fault("External event options require both query and observation callbacks");
    }
    return;
  }
  if (!external.next_ticks || !external.observe || external.maximum_events == 0 ||
      (external.order != DspExternalOrder::reject_tie &&
       external.order != DspExternalOrder::dsp_before_external &&
       external.order != DspExternalOrder::external_before_dsp)) {
    throw Fault(
        "External events require a query, observation, positive budget and explicit tie order");
  }
  if (state_.external_bound_) {
    if (external.order != state_.external_order_ ||
        query_external(machine, external) != state_.next_external_) {
      throw Fault("External event source changed its pinned deadline or tie order");
    }
    return;
  }
  if (state_.advanced_) { throw Fault("External event source must bind on the first advance"); }
  state_.external_bound_ = true;
  state_.external_order_ = external.order;
  state_.next_external_ = query_external(machine, external);
}

bool DspScheduledRun::external_first(const std::optional<std::uint64_t> native_end) const {
  if (!state_.next_external_) { return false; }
  if (!native_end || *state_.next_external_ < *native_end) { return true; }
  if (*state_.next_external_ > *native_end) { return false; }
  if (state_.external_order_ == DspExternalOrder::reject_tie) {
    throw Fault("Unqualified simultaneous external event and DSP completion");
  }
  return state_.external_order_ == DspExternalOrder::external_before_dsp;
}

void DspScheduledRun::complete_external(DspMachine &machine, const DspExternalEvents &external) {
  if (!state_.next_external_ ||
      state_.external_events_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP external event completion requires a representable pending event");
  }
  const auto ticks = *state_.next_external_;
  const auto observe = external.observe;
  const auto expected = machine.cpu().debug_state();
  observe(ticks);
  verify(state_, machine);
  if (machine.cpu().debug_state() != expected) {
    throw Fault("External event observation changed the native CPU state");
  }
  state_.last_external_ = ticks;
  ++state_.external_events_;
  state_.next_external_ = query_external(machine, external);
}
}
