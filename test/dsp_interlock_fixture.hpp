// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp_norm_fixture.hpp"

namespace blofeld::test::interlocks {
inline DspRunContract contract(const bool dependencies = true) {
  constexpr unsigned extal = 12000000;
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = extal,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = dependencies ? DspRunTiming::native_interlock_estimates
                                 : DspRunTiming::native_internal_estimates,
          .resets = DspResetTiming::reference_instruction_boundary};
}

inline void initialize(DspMachine &machine, const DspProgram &program) {
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
}

void state_checks();
}
