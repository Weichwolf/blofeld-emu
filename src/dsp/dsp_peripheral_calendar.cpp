// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_peripheral_calendar.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
PeripheralCalendarCheckpoint::PeripheralCalendarCheckpoint(PeripheralCalendarState state,
                                                           const PeripheralCalendar *owner,
                                                           std::weak_ptr<const bool> lifetime)
    : state_(std::move(state)), owner_(owner), lifetime_(std::move(lifetime)) {}

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

void PeripheralCalendar::future(const PeripheralCalendarState &state,
                                const std::optional<std::uint64_t> observed) {
  if (!observed) { return; }
  if (*observed < state.horizon.ticks || (state.last_native && *observed <= *state.last_native) ||
      (state.last_external && *observed < *state.last_external) ||
      (state.last_peripheral && *observed <= *state.last_peripheral) ||
      (state.last_peer && *observed < *state.last_peer) ||
      (state.advanced && *observed == state.horizon.ticks && state.horizon.events_through)) {
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
  auto staged = state_;
  staged.identity = identity;
  staged.pending = observed;
  future(staged, observed);
  state_ = std::move(staged);
}

void PeripheralCalendar::verify(const std::shared_ptr<const PeripheralEventIdentity> &identity,
                                const std::optional<std::uint64_t> observed) const {
  source(state_, identity, observed);
}

bool PeripheralCalendar::first(const std::optional<std::uint64_t> native,
                               const std::optional<std::uint64_t> external,
                               const PeripheralCalendarBoundary target) const {
  if (!state_.identity) { fail("query requires a bound source"); }
  horizon(state_, target);
  future(state_, state_.pending);
  const auto next = state_.pending;
  if (!next || (external && *external <= *next) || (native && *native < *next)) { return false; }
  const bool due = *next < target.ticks || (*next == target.ticks && target.events_through);
  if (!due && native == next && *next == target.ticks && target.native_through) {
    fail("horizon contradicts input-before-native order");
  }
  return due;
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
  if (state.causes == std::numeric_limits<std::uint64_t>::max()) {
    fail("cause counter exhausted");
  }
}

void PeripheralCalendar::record_cause(PeripheralCalendarState &state,
                                      const PeripheralCalendarCause cause,
                                      const std::uint64_t ticks) {
  switch (cause) {
    case PeripheralCalendarCause::native:
      if ((state.last_native && ticks == *state.last_native) ||
          (state.pending && *state.pending <= ticks)) {
        fail("native work skipped a peripheral event");
      }
      state.last_native = ticks;
      break;
    case PeripheralCalendarCause::external:
      if ((state.last_native && ticks == *state.last_native) ||
          (state.last_peripheral && ticks == *state.last_peripheral) ||
          (state.pending && *state.pending < ticks)) {
        fail("inherited event violates causal order");
      }
      state.last_external = ticks;
      break;
    case PeripheralCalendarCause::peripheral:
      if (state.pending != ticks || (state.last_native && ticks == *state.last_native)) {
        fail("peripheral dispatch requires its exact offered time");
      }
      if (state.peripheral_events == std::numeric_limits<std::uint64_t>::max()) {
        fail("event counter exhausted");
      }
      ++state.peripheral_events;
      state.last_peripheral = ticks;
      break;
    case PeripheralCalendarCause::peer:
      if ((state.last_peer && ticks == *state.last_peer) ||
          (state.pending && *state.pending < ticks)) {
        fail("peer write skipped earlier work");
      }
      state.last_peer = ticks;
      break;
    default: fail("invalid cause kind");
  }
}

void PeripheralCalendar::caused(const PeripheralCalendarCause cause,
                                const std::uint64_t ticks,
                                const std::optional<std::uint64_t> observed) {
  if (!state_.identity) { fail("cause requires a bound source"); }
  auto staged = state_;
  causal_time(staged, cause, ticks);
  record_cause(staged, cause, ticks);
  const bool existing_peer_event =
      observed.has_value() && cause == PeripheralCalendarCause::peer && state_.pending == observed;
  const bool same_time_allowed = cause == PeripheralCalendarCause::external || existing_peer_event;
  if (observed && (*observed < ticks || (*observed == ticks && !same_time_allowed))) {
    fail("new deadline precedes its actual cause");
  }
  ++staged.causes;
  staged.pending = observed;
  future(staged, observed);
  state_ = std::move(staged);
}

PeripheralCalendarState PeripheralCalendar::completed(const PeripheralCalendarState &state,
                                                      const PeripheralCalendarBoundary target,
                                                      const bool advance) {
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
  auto staged = state;
  staged.horizon = target;
  staged.advanced = staged.advanced || advance;
  future(staged, staged.pending);
  return staged;
}

void PeripheralCalendar::finish(const PeripheralCalendarBoundary target) {
  state_ = completed(state_, target, true);
}

PeripheralCalendarState PeripheralCalendar::state() const {
  return state_;
}

PeripheralCalendarCheckpoint PeripheralCalendar::checkpoint() const {
  if (!state_.identity) { fail("checkpoint requires a bound source"); }
  static_cast<void>(completed(state_, state_.horizon, false));
  return {state_, this, lifetime_};
}

void PeripheralCalendar::restored_source(
    const PeripheralCalendarCheckpoint &checkpoint,
    const std::shared_ptr<const PeripheralEventIdentity> &identity,
    const std::optional<std::uint64_t> observed,
    const PeripheralCalendarBoundary restored) {
  if (restored != checkpoint.state_.horizon) {
    fail("restored native boundary does not match checkpoint");
  }
  source(checkpoint.state_, identity, observed);
}

void PeripheralCalendar::restore(const PeripheralCalendarCheckpoint &checkpoint,
                                 const std::shared_ptr<const PeripheralEventIdentity> &identity,
                                 const std::optional<std::uint64_t> observed,
                                 const PeripheralCalendarBoundary restored) {
  if (checkpoint.owner_ != this || checkpoint.lifetime_.lock() != lifetime_) {
    fail("restore requires the original live calendar owner");
  }
  restored_source(checkpoint, identity, observed, restored);
  state_ = checkpoint.state_;
}

PeripheralCalendar::PeripheralCalendar(
    const PeripheralCalendarCheckpoint &checkpoint,
    const std::shared_ptr<const PeripheralEventIdentity> &identity,
    const std::optional<std::uint64_t> observed,
    const PeripheralCalendarBoundary restored)
    : state_(checkpoint.state_) {
  if (checkpoint.owner_ == this && !checkpoint.lifetime_.expired()) {
    fail("clone requires a distinct target");
  }
  restored_source(checkpoint, identity, observed, restored);
}

void PeripheralCalendar::reset_unbound(const PeripheralCalendarBoundary restored) {
  state_ = PeripheralCalendarState{.horizon = restored, .identity = {}};
}

}
