// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_writeback_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <variant>

namespace blofeld::test::overlap {
inline constexpr auto execution = writeback::execution;
inline constexpr std::uint64_t unit = std::uint64_t{1} << word_bits;
inline constexpr unsigned half = 0x400000;
inline constexpr std::uint64_t quarter = std::uint64_t{1} << 45U;

inline DspReferenceProgress await_retirement(DspMachine &machine, DspReferenceProgress progress) {
  constexpr unsigned limit = 20;
  for (unsigned count = 0; count < limit; ++count) {
    if (progress.pending &&
        std::holds_alternative<DspInstructionRetire>(progress.pending->effect)) {
      return progress;
    }
    require(progress.pending && !progress.retired, "Expected a held actual native result");
    progress = machine.advance_reference_access(reference::ticket(progress));
  }
  throw Fault("Own overlap effect budget exhausted");
}

inline DspReferenceTicket previous_ticket(const DspMachine &machine) {
  const auto pending = machine.reference_retirement();
  if (!pending || !std::holds_alternative<DspInstructionRetire>(pending->effect)) {
    throw Fault("Expected an actual separately owned predecessor result");
  }
  return pending->ticket;
}

void clock_checks();
}
