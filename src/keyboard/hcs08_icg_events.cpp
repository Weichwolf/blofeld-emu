// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_icg_events.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace flags = hcs08_icg_events;

namespace {
[[noreturn]] void fail(const ExecutionContext context,
                       const unsigned address,
                       const std::string &operation,
                       const std::string &reason) {
  throw Fault("HCS08 ICG fault-latch " + operation + " address=" + hex(address) +
              " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr) + " agent=CPU: " + reason);
}

unsigned event_sources(const Hcs08IcgFault event, const ExecutionContext context) {
  switch (event) {
    case Hcs08IcgFault::loss_of_lock: return flags::loss_lock;
    case Hcs08IcgFault::loss_of_clock: return flags::loss_clock;
    case Hcs08IcgFault::loss_of_clock_and_lock: return flags::both_sources;
    case Hcs08IcgFault::unqualified:
      fail(context, flags::status_address, "event", "unqualified clock detector input");
    default: fail(context, flags::status_address, "event", "unsupported clock detector input");
  }
}
}

Hcs08IcgEvents::Hcs08IcgEvents(const Hcs08IcgEventContract contract) {
  if (contract != Hcs08IcgEventContract::candidate_gt8a_fault_latches) {
    fail(context_,
         flags::status_address,
         "configure",
         "requires explicit candidate fault-latch contract");
  }
}

void Hcs08IcgEvents::context(const ExecutionContext value) {
  context_ = value;
}

void Hcs08IcgEvents::reset() {
  state_ = {};
}

void Hcs08IcgEvents::control2(const std::uint8_t value) {
  const unsigned requested = ((value & flags::reset_on_lock) != 0 ? flags::loss_lock : 0U) |
                             ((value & flags::reset_on_clock) != 0 ? flags::loss_clock : 0U);
  const unsigned changed = requested ^ static_cast<unsigned>(state_.reset_sources);
  if ((changed & state_.sources) != 0) {
    fail(context_, flags::control_address, "write", "pending-fault routing change is unqualified");
  }
  state_.reset_sources = static_cast<std::uint8_t>(requested);
}

unsigned Hcs08IcgEvents::interrupt_sources() const {
  return static_cast<unsigned>(state_.sources) & ~static_cast<unsigned>(state_.reset_sources);
}

unsigned Hcs08IcgEvents::flag_value() const {
  return ((state_.sources & flags::loss_lock) != 0 ? flags::lock_status : 0U) |
         ((state_.sources & flags::loss_clock) != 0 ? flags::clock_status : 0U) |
         (interrupt_sources() != 0 ? flags::interrupt_flag : 0U);
}

void Hcs08IcgEvents::signal(const Hcs08IcgFault event) {
  const unsigned sources = event_sources(event, context_);
  if ((sources & ~static_cast<unsigned>(state_.reset_sources)) != 0) {
    state_.interrupt_read_armed = false;
  }
  state_.sources |= static_cast<std::uint8_t>(sources);
}

Hcs08IcgFlagBits Hcs08IcgEvents::read_status_flags() {
  state_.interrupt_read_armed = interrupt_sources() != 0;
  return {.value = flag_value()};
}

void Hcs08IcgEvents::write_status(const std::uint8_t value) {
  if ((value & flags::interrupt_flag) == 0 || !state_.interrupt_read_armed) { return; }
  state_.sources &= state_.reset_sources;
  state_.interrupt_read_armed = false;
}

Hcs08IcgEventState Hcs08IcgEvents::state() const {
  return state_;
}

Hcs08IcgFaultAction Hcs08IcgEvents::action() const {
  if ((state_.sources & state_.reset_sources) != 0) { return Hcs08IcgFaultAction::reset; }
  return interrupt_sources() != 0 ? Hcs08IcgFaultAction::interrupt : Hcs08IcgFaultAction::none;
}
} // namespace blofeld
