// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <variant>

namespace blofeld::test::address_phase {
inline constexpr auto execution = DspReferenceExecution::address_and_retirement_boundaries;
inline constexpr unsigned initial_r = 7;
inline constexpr unsigned offset = 3;
inline constexpr unsigned decremented = 6;

inline DspAddressUpdate update(const DspReferenceProgress &progress) {
  if (!progress.pending || progress.retired) {
    throw Fault("Expected a pending native address update");
  }
  const auto *result = std::get_if<DspAddressUpdate>(&progress.pending->effect);
  if (result == nullptr || progress.pending->execution != execution) {
    throw Fault("Expected the explicit native memory-address phase contract");
  }
  return *result;
}

inline DspReferenceProgress retire(DspMachine &machine, DspReferenceProgress progress) {
  require(progress.pending &&
              std::holds_alternative<DspInstructionRetire>(progress.pending->effect),
          "The actual native result remains held after its last real data access");
  return machine.advance_reference_access(reference::ticket(progress));
}

void clock_checks();
}
