// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"

#include <ostream>

namespace blofeld::test {
// Own caller composition, without claiming original scheduler or DMA timing.
struct VoiceMaster {
  unsigned entry = 0;
  unsigned copied = 0;
  unsigned mixed = 0;
  unsigned filtered = 0;

  void append(DspProgram &program);
  static void initialize(DspBus &bus);
  void observe(DspBus &bus, const DspState &state, std::ostream &trace) const;
};
}
