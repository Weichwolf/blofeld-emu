// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_run_clock.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"

#include <cstdint>
#include <limits>
#include <string>

namespace blofeld {
Hcs08RunClock::Hcs08RunClock(Bus &bus, Hcs08Bdc &bdc, const Hcs08RunContract contract)
    : bus_(bus), bdc_(bdc) {
  if (contract != Hcs08RunContract::reference_instruction_retirement) {
    fail("requires the explicit unstretched instruction-retirement reference contract");
  }
  static_cast<void>(bdc_.checkpoint());
}

void Hcs08RunClock::healthy() const {
  if (!healthy_ || bdc_.faulted()) { fail("incomplete execution requires coordinated recovery"); }
}

void Hcs08RunClock::check_reservation() const {
  const auto &owned = reservation_;
  if (state_.remaining_periods == 0) {
    if (owned || bdc_.instruction_reserved()) {
      fail("CPU reservation ownership changed during an instruction boundary");
    }
    return;
  }
  if (bdc_.state().active) { fail("BDC halt during an uncommitted instruction is unqualified"); }
  if (!owned || !owned->matches_context(state_.pending, state_.code_revision) ||
      !bdc_.reservation_matches(*owned)) {
    fail("CPU state or fetched code changed during an uncommitted instruction, or its reservation "
         "owner changed");
  }
}

[[noreturn]] void Hcs08RunClock::fail(const std::string &reason) const {
  throw Fault("HCS08 run clock operation pc=" +
              (state_.pending.pc_known ? hex(state_.pending.pc) : "unknown pc-known=0") +
              " ccr=" + hex(state_.pending.ccr) +
              " offered-periods=" + std::to_string(state_.offered_periods) +
              " executing-periods=" + std::to_string(state_.executing_periods) + ": " + reason);
}

void Hcs08RunClock::advance() {
  healthy();
  healthy_ = false;
  if (bdc_.irq_bus_state()) {
    state_.pending = bdc_.cpu_state();
    fail("IRQ bus sequence/prefetch belongs to its separate qualified clock owner");
  }
  if (state_.offered_periods == std::numeric_limits<std::uint64_t>::max()) {
    fail("period counter exhausted");
  }
  if (bdc_.state().active) {
    ++state_.offered_periods;
    if (state_.remaining_periods != 0 || reservation_ || bdc_.instruction_reserved()) {
      fail("BDC halt during an uncommitted instruction is unqualified");
    }
    healthy_ = true;
    return;
  }
  if (state_.remaining_periods == 0) {
    state_.pending = bdc_.cpu_state();
    check_reservation();
    reservation_ =
        bdc_.reserve_instruction(Hcs08InstructionContract::reference_instruction_retirement);
    if (!reservation_) {
      fail("instruction not admitted; pending IRQ or another CPU owner must be handled before "
           "fetch");
    }
    ++state_.offered_periods;
    if (!state_.pending.pc_known) { fail("unknown CPU PC before instruction timing/fetch"); }
    const auto timing =
        hcs08_instruction_timing(bus_, {.pc = state_.pending.pc, .ccr = state_.pending.ccr});
    state_.remaining_periods = timing.periods;
    state_.code_revision = bus_.code_revision();
  } else {
    ++state_.offered_periods;
  }
  check_reservation();
  ++state_.executing_periods;
  if (--state_.remaining_periods == 0) {
    const auto &owned = reservation_;
    if (!owned) { fail("completed budget lost its instruction reservation"); }
    bdc_.retire_reserved_instruction(*owned);
    reservation_.reset();
    ++state_.retired_instructions;
  }
  healthy_ = true;
}

Hcs08RunState Hcs08RunClock::state() const {
  return state_;
}

bool Hcs08RunClock::faulted() const {
  return !healthy_ || bdc_.faulted();
}

Hcs08RunCheckpoint Hcs08RunClock::checkpoint() const {
  healthy();
  static_cast<void>(bdc_.checkpoint());
  check_reservation();
  Hcs08RunCheckpoint checkpoint;
  checkpoint.state_ = state_;
  checkpoint.reservation_ = reservation_;
  return checkpoint;
}

void Hcs08RunClock::restore_checkpoint(const Hcs08RunCheckpoint &checkpoint) {
  static_cast<void>(bdc_.checkpoint());
  healthy_ = false;
  state_ = checkpoint.state_;
  reservation_ = checkpoint.reservation_;
  check_reservation();
  healthy_ = true;
}
} // namespace blofeld
