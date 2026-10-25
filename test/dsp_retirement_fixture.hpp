// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <variant>

namespace blofeld::test::retirement {
inline constexpr auto execution = DspReferenceExecution::access_and_retirement_boundaries;

inline const DspInstructionRetire &result(const DspReferenceProgress &progress) {
  if (progress.retired || !progress.pending) { throw Fault("Expected a held native result"); }
  const auto *retire = std::get_if<DspInstructionRetire>(&progress.pending->effect);
  if (retire == nullptr || progress.pending->execution != execution) {
    throw Fault("Expected a genuine retirement marker under its explicit execution contract");
  }
  return *retire;
}

inline DspReferenceProgress complete_effects(DspMachine &machine, DspReferenceProgress progress) {
  constexpr unsigned budget = 16;
  for (unsigned index = 0; index < budget; ++index) {
    if (progress.pending &&
        std::holds_alternative<DspInstructionRetire>(progress.pending->effect)) {
      return progress;
    }
    require(!progress.retired, "Explicit retirement cannot be published by the last bus effect");
    progress = machine.advance_reference_access(reference::ticket(progress));
  }
  throw Fault("Own deferred instruction exceeded its native effect bound");
}

void fetch_checks();
}
