// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_peripheral_calendar.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace blofeld {
PeripheralCalendar::PeripheralCalendar(const PeripheralCalendarBoundary initial) {
  state_.horizon = initial;
}

[[noreturn]] void PeripheralCalendar::fail(const char *reason) {
  throw Fault(std::string("Peripheral calendar: ") + reason);
}

void PeripheralCalendar::horizon(const PeripheralCalendarState &state,
                                 const PeripheralCalendarBoundary next) {
  const auto old = state.horizon;
  if (next.ticks < old.ticks || (state.advanced && next.ticks == old.ticks &&
                                 ((old.native_through && !next.native_through) ||
                                  (old.events_through && !next.events_through)))) {
    fail("consumed horizon cannot rewind");
  }
}

void PeripheralCalendar::deadline_horizon(const PeripheralCalendarBoundary boundary,
                                          const bool advanced,
                                          const std::optional<std::uint64_t> observed) {
  if (observed && (*observed < boundary.ticks ||
                   (advanced && *observed == boundary.ticks && boundary.events_through))) {
    fail("deadline precedes a consumed boundary");
  }
}

void PeripheralCalendar::future(const PeripheralCalendarState &state,
                                const std::optional<std::uint64_t> observed) {
  deadline_horizon(state.horizon, state.advanced, observed);
  if (observed && ((state.last_native && *observed <= *state.last_native) ||
                   (state.last_external && *observed < *state.last_external) ||
                   (state.last_peripheral && *observed <= *state.last_peripheral) ||
                   (state.last_peer && *observed < *state.last_peer))) {
    fail("deadline precedes a consumed boundary");
  }
}

void PeripheralCalendar::source(const PeripheralCalendarState &state,
                                const std::shared_ptr<const PeripheralEventIdentity> &identity,
                                const std::optional<std::uint64_t> observed) {
  if (!identity || identity != state.identity || observed != state.pending) {
    fail("source identity or pinned pending deadline changed");
  }
  future(state, observed);
}

void PeripheralCalendar::bind(const std::shared_ptr<const PeripheralEventIdentity> &identity,
                              const std::optional<std::uint64_t> observed) {
  if (state_.identity) {
    verify(identity, observed);
    return;
  }
  if (!identity) { fail("source requires a stable identity"); }
  future(state_, observed);
  state_.identity = identity;
  state_.pending = observed;
}

void PeripheralCalendar::verify(const std::shared_ptr<const PeripheralEventIdentity> &identity,
                                const std::optional<std::uint64_t> observed) const {
  source(state_, identity, observed);
}

void PeripheralCalendar::causal_time(const PeripheralCalendarState &state,
                                     const PeripheralCalendarCause cause,
                                     const std::uint64_t ticks) {
  if (ticks < state.horizon.ticks ||
      (state.advanced && ticks == state.horizon.ticks && cause != PeripheralCalendarCause::peer &&
       (cause == PeripheralCalendarCause::native ? state.horizon.native_through
                                                 : state.horizon.events_through)) ||
      (state.last_native && ticks < *state.last_native) ||
      (state.last_external && ticks < *state.last_external) ||
      (state.last_peripheral && ticks < *state.last_peripheral) ||
      (state.last_peer && ticks < *state.last_peer)) {
    fail("cause precedes a consumed boundary");
  }
}

std::optional<std::uint64_t> &PeripheralCalendar::cause_stamp(PeripheralCalendarState &state,
                                                              const PeripheralCalendarCause cause,
                                                              const std::uint64_t ticks) {
  switch (cause) {
    case PeripheralCalendarCause::native:
      if ((state.last_native && ticks == *state.last_native) ||
          (state.pending && *state.pending <= ticks)) {
        fail("native work skipped a peripheral event");
      }
      return state.last_native;
    case PeripheralCalendarCause::external:
      if ((state.last_native && ticks == *state.last_native) ||
          (state.last_peripheral && ticks == *state.last_peripheral) ||
          (state.pending && *state.pending < ticks)) {
        fail("inherited event violates causal order");
      }
      return state.last_external;
    case PeripheralCalendarCause::peripheral:
      if (state.pending != ticks || (state.last_native && ticks == *state.last_native)) {
        fail("peripheral dispatch requires its exact offered time");
      }
      return state.last_peripheral;
    case PeripheralCalendarCause::peer:
      if ((state.last_peer && ticks == *state.last_peer) ||
          (state.pending && *state.pending < ticks)) {
        fail("peer write skipped earlier work");
      }
      return state.last_peer;
    default: fail("invalid cause kind");
  }
}

void PeripheralCalendar::caused(const PeripheralCalendarCause cause,
                                const std::uint64_t ticks,
                                const std::optional<std::uint64_t> observed) {
  if (!state_.identity) { fail("cause requires a bound source"); }
  causal_time(state_, cause, ticks);
  auto &last = cause_stamp(state_, cause, ticks);
  const bool existing_peer_event =
      observed.has_value() && cause == PeripheralCalendarCause::peer && state_.pending == observed;
  const bool peer_before_inputs = cause == PeripheralCalendarCause::peer &&
                                  state_.horizon.ticks == ticks && !state_.horizon.events_through &&
                                  (!state_.last_native || *state_.last_native < ticks);
  const bool same_time_allowed =
      cause == PeripheralCalendarCause::external || existing_peer_event || peer_before_inputs;
  if (observed && (*observed < ticks || (*observed == ticks && !same_time_allowed))) {
    fail("new deadline precedes its actual cause");
  }
  future(state_, observed);
  last = ticks;
  state_.pending = observed;
}

void PeripheralCalendar::finish(const PeripheralCalendarBoundary target) {
  const auto &state = state_;
  if (!state.identity) { fail("finish requires a bound source"); }
  horizon(state, target);
  if ((state.last_native && (*state.last_native > target.ticks ||
                             (*state.last_native == target.ticks && !target.native_through))) ||
      (state.last_external && (*state.last_external > target.ticks ||
                               (*state.last_external == target.ticks && !target.events_through))) ||
      (state.last_peripheral &&
       (*state.last_peripheral > target.ticks ||
        (*state.last_peripheral == target.ticks && !target.events_through)))) {
    fail("horizon would omit completed work");
  }
  if (state.last_peer && *state.last_peer > target.ticks) { fail("horizon would omit peer work"); }
  future(state, state.pending);
  deadline_horizon(target, true, state.pending);
  state_.horizon = target;
  state_.advanced = true;
}

PeripheralCalendarState PeripheralCalendar::state() const {
  return state_;
}

}
