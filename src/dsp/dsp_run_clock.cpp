// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_run_clock.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_pll.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <string>

namespace blofeld {
namespace {
std::string context_text(const DspRunContext context) {
  return " pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6);
}
}

DspRunClock::DspRunClock(const DspRunClockInputs inputs) : inputs_(inputs) {
  constexpr unsigned minimum_extal = 500000;
  constexpr unsigned maximum_extal = 100000000;
  constexpr std::uint64_t maximum_timebase = 1000000000000;
  if (inputs.switch_timing != DspClockSwitch::reference_instruction_boundary ||
      inputs.extal_hz < minimum_extal || inputs.extal_hz > maximum_extal ||
      inputs.timebase_hz > maximum_timebase || inputs.timebase_hz < inputs.extal_hz ||
      inputs.timebase_hz % inputs.extal_hz != 0) {
    throw Fault("DSP run clock requires an explicit integral EXTAL/reference-timebase contract");
  }
}

std::uint64_t DspRunClock::period_ticks(const DspBus &bus) const {
  const auto ratio = bus.pll().target_ratio();
  const auto numerator = inputs_.timebase_hz * ratio.denominator;
  const auto denominator = static_cast<std::uint64_t>(inputs_.extal_hz) * ratio.numerator;
  if (numerator % denominator != 0 || numerator < denominator) {
    throw Fault("DSP PLL target is not integral in the declared reference timebase");
  }
  return numerator / denominator;
}

std::uint64_t DspRunClock::duration_ticks(const DspClockUnits units,
                                          const std::uint64_t period) const {
  constexpr unsigned maximum_step_cycles = 1048576;
  const auto remaining = std::numeric_limits<std::uint64_t>::max() - ticks_;
  if (units.native == 0 || units.native > maximum_step_cycles ||
      units.interlocks > maximum_step_cycles - units.native) {
    throw Fault("Invalid or exhausted DSP reference cycle count");
  }
  const auto elapsed = units.native + units.interlocks;
  if (elapsed > std::numeric_limits<std::uint64_t>::max() - cycles_ - interlocks_ ||
      elapsed > remaining / period) {
    throw Fault("Invalid or exhausted DSP reference cycle count");
  }
  return static_cast<std::uint64_t>(elapsed) * period;
}

std::uint64_t DspRunClock::next_step_ticks(const DspBus &bus, const unsigned native_cycles) const {
  return next_step_ticks(bus, DspClockUnits{.native = native_cycles});
}

std::uint64_t DspRunClock::next_step_ticks(const DspBus &bus, const DspClockUnits units) const {
  if (faulted_ || active_) { throw Fault("DSP duration preview requires a healthy step boundary"); }
  return duration_ticks(units, period_ticks(bus));
}

void DspRunClock::begin_step(const DspBus &bus, const DspRunContext context) {
  if (faulted_ || active_) { throw Fault("DSP reference clock requires a healthy step boundary"); }
  context_ = context;
  try {
    period_ticks_ = period_ticks(bus);
    active_ = true;
  } catch (const std::exception &error) {
    fail();
    throw Fault("DSP clock prepare" + context_text(context) + ": " + error.what());
  }
}

void DspRunClock::finish_step(DspBus &bus, const unsigned native_cycles) {
  finish_step(bus, DspClockUnits{.native = native_cycles});
}

void DspRunClock::finish_step(DspBus &bus, const DspClockUnits units) {
  if (faulted_ || !active_) { throw Fault("DSP reference clock has no healthy prepared step"); }
  try {
    const auto ticks = duration_ticks(units, period_ticks_);
    const auto elapsed = units.native + units.interlocks;
    const auto phase = (epoch_native_cycle_units() + epoch_interlock_cycle_units()) & 1U;
    const auto edges = (phase + elapsed) / 2;
    bus.advance_instruction_cycles(elapsed);
    bus.timers().advance_clk2(edges);
    ticks_ += ticks;
    cycles_ += units.native;
    interlocks_ += units.interlocks;
    clk2_edges_ += edges;
    active_ = false;
  } catch (const std::exception &error) {
    fail();
    throw Fault("DSP clock finish" + context_text(context_) +
                " completed-native-cycle-units=" + std::to_string(cycles_) + ": " + error.what());
  }
}

void DspRunClock::restart_native_epoch() {
  if (faulted_ || active_ || epoch_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP reset epoch requires a healthy inactive representable boundary");
  }
  epoch_start_ = cycles_;
  epoch_interlock_start_ = interlocks_;
  ++epoch_;
  period_ticks_ = 0;
  context_ = {};
}

void DspRunClock::fail() noexcept {
  faulted_ = true;
  active_ = false;
}

}
