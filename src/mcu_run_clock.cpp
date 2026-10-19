// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_run_clock.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clock.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <numeric>
#include <string>

namespace blofeld {
namespace {
std::uint32_t oscillator_rate(const McuRunClockInputs inputs) {
  constexpr unsigned minimum_hz = 500000;
  constexpr unsigned maximum_hz = 40000000;
  if (inputs.switch_timing != McuRunClockSwitch::reference_cycle_boundary) {
    throw Fault("Explicit reference-cycle clock switching contract is required");
  }
  if (inputs.oscillator_hz < minimum_hz || inputs.oscillator_hz > maximum_hz) {
    throw Fault("MC9S12UF32 reference oscillator must be between 500000 and 40000000 Hz");
  }
  return inputs.oscillator_hz;
}

std::uint64_t initial_phase(const McuRunClockInputs inputs, const std::uint64_t period) {
  if (inputs.initial_oscillator_phase_ticks >= period) {
    throw Fault("Initial oscillator phase must be smaller than its exact reference period");
  }
  return inputs.initial_oscillator_phase_ticks;
}
}

McuRunClock::McuRunClock(const McuRunClockInputs inputs)
    : oscillator_hz_(oscillator_rate(inputs)),
      timebase_hz_(
          std::lcm(static_cast<std::uint64_t>(oscillator_hz_), static_cast<std::uint64_t>(phy_hz))),
      oscillator_period_(timebase_hz_ / oscillator_hz_),
      oscillator_phase_(initial_phase(inputs, oscillator_period_)) {}

void McuRunClock::advance_run_cycle(McuBus &bus, const ExecutionContext context) {
  if (faulted_) { throw Fault("MCU run clock requires coordinated recovery after a failed cycle"); }
  bus.begin_instruction(context);
  try {
    const auto route = bus.run_clock_route();
    if (route.source == McuClockSource::usb_phy && oscillator_hz_ != nominal_oscillator_hz) {
      throw Fault("Unqualified 60 MHz PHY with non-12 MHz reference oscillator: " +
                  std::to_string(oscillator_hz_));
    }
    const auto source_hz = route.source == McuClockSource::usb_phy ? phy_hz : oscillator_hz_;
    const auto ticks = (timebase_hz_ / source_hz) * route.bus_divider;
    const auto oscillator_total = oscillator_phase_ + ticks;
    const auto edges = oscillator_total / oscillator_period_;
    if (ticks > std::numeric_limits<std::uint64_t>::max() - elapsed_ticks_ ||
        bus_cycles_ == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("MCU run clock counter exhausted");
    }
    bus.advance_sci(1);
    bus.advance_timer(1);
    bus.advance_oscillator(static_cast<unsigned>(edges));
    elapsed_ticks_ += ticks;
    oscillator_phase_ = oscillator_total % oscillator_period_;
    oscillator_cycles_ += edges;
    ++bus_cycles_;
  } catch (const std::exception &error) {
    faulted_ = true;
    throw Fault("MCU run clock advance pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) +
                " completed-bus-cycles=" + std::to_string(bus_cycles_) +
                " timebase-ticks=" + std::to_string(elapsed_ticks_) + ": " + error.what());
  }
}

}
