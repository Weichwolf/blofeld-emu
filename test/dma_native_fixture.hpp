// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::dma_test {
inline constexpr unsigned boot = 0xff0000;
inline constexpr DspRunClockInputs native_clocks{.extal_hz = 12000000,
                                                 .timebase_hz = 540000000,
                                                 .switch_timing =
                                                     DspClockSwitch::reference_cycle_boundary};

class NativeProgram {
public:
  void word(std::uint32_t value);
  void write(DspStore store);
  [[nodiscard]] unsigned pc() const;
  void install(DspBus &bus) const;

private:
  std::vector<std::uint32_t> words_;
};

void reset_native(DspMachine &machine, const NativeProgram &program);
void step_native(DspMachine &machine);
// Explicit standalone reference phases with the native CPU held. These do not
// supply automatic instruction timing or physical ESAI/core synchronization.
[[nodiscard]] std::optional<DspDmaWord> dma(DspMachine &machine, DspDmaInputs inputs);
}
