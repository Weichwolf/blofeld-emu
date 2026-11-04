// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
std::optional<std::uint64_t> DspRunClock::next_native_boundary_ticks(const DspBus &bus) const {
  if (inputs_.switch_timing != DspClockSwitch::functional_region_boundary || faulted_ || active_ ||
      hardware_reset_active()) {
    throw Fault("DSP native horizon requires a healthy functional clock boundary");
  }
  const auto phase =
      static_cast<unsigned>((epoch_native_cycle_units() + epoch_interlock_cycle_units()) & 1U);
  const auto cycles = bus.native_cycle_boundary(phase);
  if (!cycles) { return std::nullopt; }
  const auto period = period_ticks(bus);
  if (*cycles > (std::numeric_limits<std::uint64_t>::max() - ticks_) / period) {
    throw Fault("DSP native event horizon exhausted common time");
  }
  return *cycles * period;
}

void DspRunClock::finish_native_region(DspBus &bus,
                                       const unsigned cycles,
                                       const DspDmaInputs inputs) {
  finish_native_region(bus, DspClockUnits{.native = cycles}, inputs);
}

void DspRunClock::finish_native_region(DspBus &bus,
                                       const DspClockUnits units,
                                       const DspDmaInputs inputs) {
  if (inputs_.switch_timing != DspClockSwitch::functional_region_boundary || faulted_ || !active_) {
    throw Fault("DSP native region requires its healthy prepared functional clock");
  }
  try {
    const auto duration = duration_ticks(units, period_ticks_);
    const auto cycles = units.native + units.interlocks;
    if (period_ticks(bus) != period_ticks_) {
      throw Fault("DSP clock command changed inside a native region");
    }
    const auto phase =
        static_cast<unsigned>((epoch_native_cycle_units() + epoch_interlock_cycle_units()) & 1U);
    const auto boundary = bus.native_cycle_boundary(phase);
    if ((boundary && cycles > *boundary) || !bus.native_dma_quiescent()) {
      throw Fault("DSP native region crossed a device event or uncompleted DMA transition");
    }
    const auto edges = (static_cast<std::uint64_t>(phase) + cycles) / 2;
    bus.advance_quiescent_cycles(cycles, inputs);
    bus.timers().advance_clk2(edges);
    ticks_ += duration;
    cycles_ += units.native;
    interlocks_ += units.interlocks;
    clk2_edges_ += edges;
    active_ = false;
  } catch (const std::exception &error) {
    fail();
    throw Fault("DSP native region pc=" + hex(context_.pc, 6) + " omr=" + hex(context_.omr, 6) +
                " completed-cycles=" + std::to_string(cycles_) + ": " + error.what());
  }
}

std::optional<DspDmaWord> DspRunClock::finish_native_cycle(DspBus &bus,
                                                           const DspDmaInputs inputs,
                                                           const DspClockUnits units) {
  if (inputs_.switch_timing != DspClockSwitch::functional_region_boundary || faulted_ || !active_) {
    throw Fault("DSP native cycle requires its healthy prepared functional clock");
  }
  try {
    static_cast<void>(duration_ticks(units, period_ticks_));
    if (units.native + units.interlocks != 1) {
      throw Fault("DSP native DMA requires exactly one elapsed period");
    }
    const bool switch_was_pending = bus.switch_delay_ != 0;
    const auto result = bus.advance_dma(inputs);
    finish_dma_cycle(bus, inputs.cycle, switch_was_pending, units);
    return result;
  } catch (const std::exception &error) {
    fail();
    throw Fault("DSP native cycle pc=" + hex(context_.pc, 6) + " omr=" + hex(context_.omr, 6) +
                " completed-cycles=" + std::to_string(cycles_) + ": " + error.what());
  }
}
}
