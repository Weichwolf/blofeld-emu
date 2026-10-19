// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_run_clock.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_pll.hpp"

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

std::uint64_t DspRunClock::duration_ticks(const unsigned native_cycles,
                                          const std::uint64_t period) const {
  constexpr unsigned maximum_step_cycles = 1048576;
  const auto remaining = std::numeric_limits<std::uint64_t>::max() - ticks_;
  if (native_cycles == 0 || native_cycles > maximum_step_cycles ||
      native_cycles > std::numeric_limits<std::uint64_t>::max() - cycles_ ||
      native_cycles > remaining / period) {
    throw Fault("Invalid or exhausted DSP reference cycle count");
  }
  return static_cast<std::uint64_t>(native_cycles) * period;
}

std::uint64_t DspRunClock::next_step_ticks(const DspBus &bus, const unsigned native_cycles) const {
  if (faulted_ || active_) { throw Fault("DSP duration preview requires a healthy step boundary"); }
  return duration_ticks(native_cycles, period_ticks(bus));
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
  if (faulted_ || !active_) { throw Fault("DSP reference clock has no healthy prepared step"); }
  try {
    const auto ticks = duration_ticks(native_cycles, period_ticks_);
    const auto edges = ((cycles_ & 1U) + native_cycles) / 2;
    bus.advance_instruction_cycles(native_cycles);
    bus.timers().advance_clk2(edges);
    ticks_ += ticks;
    cycles_ += native_cycles;
    active_ = false;
  } catch (const std::exception &error) {
    fail();
    throw Fault("DSP clock finish" + context_text(context_) +
                " completed-native-cycle-units=" + std::to_string(cycles_) + ": " + error.what());
  }
}

void DspRunClock::fail() noexcept {
  faulted_ = true;
  active_ = false;
}

}
