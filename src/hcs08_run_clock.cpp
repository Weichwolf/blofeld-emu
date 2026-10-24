// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_run_clock.hpp"

#include "blofeld/bus.hpp"
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
  if (state_.offered_periods == std::numeric_limits<std::uint64_t>::max()) {
    fail("period counter exhausted");
  }
  ++state_.offered_periods;
  if (bdc_.state().active) {
    if (state_.remaining_periods != 0) {
      fail("BDC halt during an uncommitted instruction is unqualified");
    }
    healthy_ = true;
    return;
  }
  if (state_.remaining_periods == 0) {
    state_.pending = bdc_.cpu_state();
    if (!state_.pending.pc_known) { fail("unknown CPU PC before instruction timing/fetch"); }
    const auto timing =
        hcs08_instruction_timing(bus_, {.pc = state_.pending.pc, .ccr = state_.pending.ccr});
    state_.remaining_periods = timing.periods;
    state_.code_revision = bus_.code_revision();
  }
  if (bdc_.cpu_state() != state_.pending || bus_.code_revision() != state_.code_revision) {
    fail("CPU state or fetched code changed during an uncommitted instruction");
  }
  ++state_.executing_periods;
  if (--state_.remaining_periods == 0) {
    bdc_.step_cpu();
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
  Hcs08RunCheckpoint checkpoint;
  checkpoint.state_ = state_;
  return checkpoint;
}

void Hcs08RunClock::restore_checkpoint(const Hcs08RunCheckpoint &checkpoint) {
  static_cast<void>(bdc_.checkpoint());
  state_ = checkpoint.state_;
  healthy_ = true;
}
} // namespace blofeld
