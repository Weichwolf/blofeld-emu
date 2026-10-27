// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
std::optional<std::uint64_t>
DspFetchScheduledRun::query_external(const DspFetchExternalEvents &external) const {
  const auto next = external.next_ticks();
  run_.validate_boundary(machine_);
  if (next &&
      (*next < state_.horizon.ticks || (state_.last_native && *next < *state_.last_native) ||
       (state_.last_external && *next < *state_.last_external) ||
       (state_.last_native && *next == *state_.last_native &&
        state_.order != DspExternalOrder::dsp_before_external) ||
       (!state_.next_external && state_.advanced && *next == state_.horizon.ticks &&
        state_.horizon.external == DspScheduleBoundary::through))) {
    throw Fault("DSP pipeline external timestamp precedes its consumed timeline");
  }
  return next;
}

void DspFetchScheduledRun::bind_external(const DspFetchExternalEvents &external) {
  const bool supplied = external.identity || external.next_ticks || external.observe;
  if (!supplied) {
    if (state_.external_identity || external.order != DspExternalOrder::unqualified ||
        external.maximum_events != 0) {
      throw Fault("DSP pipeline bound event source was omitted or malformed");
    }
    return;
  }
  if (!external.identity || !external.next_ticks || !external.observe ||
      external.maximum_events == 0 ||
      (external.order != DspExternalOrder::reject_tie &&
       external.order != DspExternalOrder::external_before_dsp &&
       external.order != DspExternalOrder::dsp_before_external)) {
    throw Fault("DSP pipeline events require identity,callbacks,budget and explicit tie policy");
  }
  if (state_.external_identity) {
    if (state_.external_identity != external.identity || state_.order != external.order) {
      throw Fault("DSP pipeline external source identity or tie policy changed");
    }
    const auto next = query_external(external);
    if (state_.next_external && next != state_.next_external) {
      throw Fault("DSP pipeline source changed its pinned pending deadline");
    }
    state_.next_external = next;
    return;
  }
  if (state_.advanced) { throw Fault("DSP pipeline source must bind on its first advance"); }
  state_.external_identity = external.identity;
  state_.order = external.order;
  state_.next_external = query_external(external);
}

void DspFetchScheduledRun::complete_external(const DspFetchExternalEvents &external) {
  if (!state_.next_external ||
      state_.external_events == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP pipeline external event counter exhausted or no pending event");
  }
  const auto ticks = *state_.next_external;
  external.observe(ticks);
  run_.validate_boundary(machine_);
  state_.last_external = ticks;
  ++state_.external_events;
  state_.next_external = external.next_ticks();
  run_.validate_boundary(machine_);
  if (state_.next_external && *state_.next_external < ticks) {
    throw Fault("DSP pipeline source moved backwards after an observed event");
  }
}
} // namespace blofeld
