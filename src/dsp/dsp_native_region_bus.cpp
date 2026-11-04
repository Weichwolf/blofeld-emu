// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
bool DspBus::native_dma_quiescent(const DspDmaInputs inputs) {
  return native_dma_quiescent() && dma_.request_levels_held(resolve_dma_inputs(inputs));
}

std::optional<std::uint64_t> DspBus::native_cycle_boundary(const unsigned clk2_phase) const {
  if (clk2_phase > 1) { throw Fault("Invalid native DSP CLK/2 phase"); }
  std::optional<std::uint64_t> result;
  const auto earlier = [&result](const std::uint64_t cycles) {
    if (!result || cycles < *result) { result = cycles; }
  };
  if (!native_dma_quiescent()) { earlier(1); }
  if (switch_delay_ != 0) { earlier(switch_delay_); }
  if (const auto serial = shi_.next_cycle_boundary()) { earlier(*serial); }
  if (const auto timer = timers_.next_clk2_boundary()) {
    if (*timer > std::numeric_limits<std::uint64_t>::max() / 2) {
      throw Fault("Native DSP timer horizon exceeds its cycle cursor");
    }
    earlier((*timer * 2) - clk2_phase);
  }
  return result;
}

void DspBus::advance_quiescent_cycles(const unsigned cycles, const DspDmaInputs inputs) {
  const auto before = dma_.last_cycle();
  if (!before || cycles == 0 || cycles > std::numeric_limits<std::uint64_t>::max() - *before ||
      inputs.cycle != *before + cycles) {
    throw Fault("Native DSP region lost its consecutive DMA cursor at X:0xfffff4");
  }
  dma_.advance_quiescent(resolve_dma_inputs(inputs));
  advance_core_delays(cycles);
}
}
