// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_reset.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hcs08_bdc.hpp"

#include <cstdint>
#include <limits>
#include <string>

namespace blofeld {
Hcs08ResetSequence::Hcs08ResetSequence(const Hcs08ResetContract contract) {
  if (contract != Hcs08ResetContract::reference_bus_edges) {
    fail("requires explicit nominal reset-edge reference contract");
  }
}

[[noreturn]] void Hcs08ResetSequence::fail(const std::string &reason) const {
  throw Fault("HCS08 reset RESET/MS operation pc=" + hex(context_.pc) + " ccr=" +
              hex(context_.ccr) + " reset-edges=" + std::to_string(state_.edges) + ": " + reason);
}

bool Hcs08ResetSequence::drives_low() const {
  return state_.phase == Hcs08ResetPhase::driving;
}

bool Hcs08ResetSequence::level(const Hcs08ResetDrive reset) const {
  if (reset != Hcs08ResetDrive::low && reset != Hcs08ResetDrive::released) {
    fail("unknown or unsupported external RESET driver");
  }
  return !drives_low() && reset == Hcs08ResetDrive::released;
}

void Hcs08ResetSequence::edge(const Hcs08ResetDrive reset,
                              const Hcs08ModePad mode,
                              const ExecutionContext context) {
  context_ = context;
  if (!healthy_) { fail("incomplete reset requires coordinated recovery"); }
  healthy_ = false;
  if (state_.phase == Hcs08ResetPhase::ready) { fail("reset sequence already completed"); }
  static_cast<void>(level(reset));
  if (state_.mode && reset == Hcs08ResetDrive::low) {
    fail("new external assertion after RESET rose requires a new reset owner");
  }
  constexpr unsigned drive_periods = 34;
  constexpr unsigned sample_period = 72;
  constexpr unsigned vector_periods = 6;
  if (state_.edges == std::numeric_limits<std::uint64_t>::max()) {
    fail("reset clock horizon exhausted");
  }
  ++state_.edges;
  if (state_.edges == drive_periods) { state_.phase = Hcs08ResetPhase::settling; }
  if (!state_.mode && level(reset)) {
    if (mode != Hcs08ModePad::low && mode != Hcs08ModePad::high) {
      fail("unknown BKGD/MS at the reference RESET rising boundary");
    }
    state_.mode = mode == Hcs08ModePad::low ? Hcs08BdcReset::bkgd_held_low : Hcs08BdcReset::normal;
  }
  if (state_.edges == sample_period) {
    state_.external_at_sample = !level(reset);
    state_.phase = level(reset) ? Hcs08ResetPhase::vector : Hcs08ResetPhase::external_hold;
  } else if (state_.phase == Hcs08ResetPhase::external_hold && level(reset)) {
    state_.phase = Hcs08ResetPhase::vector;
  } else if (state_.phase == Hcs08ResetPhase::vector) {
    ++state_.vector_periods;
    if (state_.vector_periods == vector_periods) { state_.phase = Hcs08ResetPhase::ready; }
  }
  healthy_ = true;
}

Hcs08ResetState Hcs08ResetSequence::state() const {
  return state_;
}

bool Hcs08ResetSequence::faulted() const {
  return !healthy_;
}
}
