// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
constexpr std::uint64_t timebase = 635040000000;
constexpr unsigned extal = 11289600;
constexpr std::uint64_t reset_period = 56250;
constexpr unsigned pll_15_over_2 = 0x04601e;
constexpr unsigned divided_pll = 0x04e11e;
constexpr unsigned nonintegral_pll = 0x04601d;
constexpr unsigned reset_command = 0x04401d;
constexpr unsigned pstp = 0x1000;
constexpr unsigned observed_pc = 0x123;
unsigned assertions = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw blofeld::Fault(message); }
}

std::string refuses(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected a visible clock refusal");
}

blofeld::DspRunClock clock() {
  return blofeld::DspRunClock(
      {.extal_hz = extal,
       .timebase_hz = timebase,
       .switch_timing = blofeld::DspClockSwitch::functional_region_boundary});
}

void reset(blofeld::DspBus &bus, const std::optional<bool> pin = false) {
  bus.pll().observe_pinit(pin);
  bus.reset();
}

void test_unknown_reset_and_failed_periods() {
  blofeld::DspBus bus;
  auto target = clock();
  require(refuses([&] { static_cast<void>(target.next_step_ticks(bus, 1)); }).contains("PINIT"),
          "Unknown reset must remain visible before any cache entry exists");
  reset(bus);
  require(target.next_step_ticks(bus, 1) == reset_period, "Known reset has literal period56250");
  require(target.next_step_ticks(bus, 1) == reset_period, "Repeated lookup preserves reset rate");
  reset(bus, std::nullopt);
  require(refuses([&] { static_cast<void>(target.next_step_ticks(bus, 1)); }).contains("PINIT"),
          "Unknown reset with the same raw command cannot reuse an earlier known period");
  require(!target.faulted(), "Pure duration preview refusal does not retire or fault a step");
  reset(bus);
  bus.pll().write(nonintegral_pll);
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    require(refuses([&] {
              static_cast<void>(target.next_step_ticks(bus, 1));
            }).contains("not integral"),
            "A failed period is never replaced by a previously cached success");
  }
  bus.pll().write(pll_15_over_2);
  constexpr std::uint64_t fast_period = 7500;
  require(target.next_step_ticks(bus, 1) == fast_period,
          "A valid command after refusal recomputes its genuine7500-tick period");
  constexpr unsigned invalid_command = 0x00601d;
  static_cast<void>(refuses([&] { bus.pll().write(invalid_command); }));
  require(bus.pll().read() == pll_15_over_2 && target.next_step_ticks(bus, 1) == fast_period,
          "A rejected PCTL write preserves the actual command and successful duration");
}

void test_all_command_fields_and_restore() {
  struct Case {
    unsigned control;
    std::uint64_t period;
  };

  // Hand-derived periods: bypass56250, enabled2*MF/(PDF*2^DF*OD0divider).
  constexpr std::array cases{Case{.control = 0x04601e, .period = 7500},
                             Case{.control = 0x04e01e, .period = 7500},
                             Case{.control = 0x04e11e, .period = 15000},
                             Case{.control = 0x04a11e, .period = 7500},
                             Case{.control = 0x08a11e, .period = 15000},
                             Case{.control = 0x08a10f, .period = 30000},
                             Case{.control = 0x08810f, .period = 112500}};
  blofeld::DspBus bus;
  reset(bus);
  auto target = clock();
  for (const auto &item : cases) {
    bus.pll().write(item.control);
    require(target.next_step_ticks(bus, 1) == item.period,
            "PEN, DF, OD, PDF and MF changes retain independently specified periods");
    require(target.next_step_ticks(bus, 2) == 2 * item.period,
            "A repeated cached period remains scaled by the requested unit count");
  }
  bus.pll().write(pll_15_over_2);
  constexpr std::uint64_t fast_period = 7500;
  constexpr std::uint64_t slow_period = 15000;
  require(target.next_step_ticks(bus, 1) == fast_period, "Warm the first restored command");
  const auto saved_bus = bus;
  const auto saved_clock = target;
  bus.pll().write(divided_pll);
  require(target.next_step_ticks(bus, 1) == slow_period, "A changed bus invalidates its key");
  target = saved_clock;
  require(target.next_step_ticks(bus, 1) == slow_period,
          "Restoring only the clock cannot reuse its old command against the changed bus");
  bus = saved_bus;
  require(target.next_step_ticks(bus, 1) == fast_period,
          "Restoring only the bus selects its actual command despite a newer cached key");
  target = saved_clock;
  require(target.next_step_ticks(bus, 1) == fast_period,
          "Paired clock and bus restore retains the original duration");
  require(target.elapsed_ticks() == 0 && target.native_cycle_units() == 0,
          "All previews and restores above leave guest clocks unadvanced");
}

blofeld::DspDmaInputs inputs(const std::uint64_t cycle) {
  return {.cycle = cycle,
          .known_requests = 15,
          .requests = 0,
          .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
          .grant = blofeld::DspDmaGrant::available};
}

void prime(blofeld::DspRunClock &target, blofeld::DspBus &bus) {
  target.begin_step(bus, {.pc = observed_pc, .omr = 0});
  require(!target.finish_native_cycle(bus, inputs(0)), "Initial DMA sample fabricates no word");
  require(bus.native_dma_quiescent() && target.elapsed_ticks() == reset_period,
          "A real initial cycle establishes a quiescent peripheral boundary");
}

void test_native_finish_detects_rate_changes() {
  blofeld::DspBus bus;
  reset(bus);
  auto target = clock();
  prime(target, bus);
  target.begin_step(bus, {.pc = observed_pc, .omr = 0});
  bus.pll().write(pll_15_over_2);
  const auto failure = refuses([&] { target.finish_native_region(bus, 1, inputs(1)); });
  require(failure.contains("command changed") && failure.contains("pc=0x000123"),
          "A changed PLL rate inside a native region remains a contextual visible failure");
  require(target.faulted() && target.elapsed_ticks() == reset_period &&
              target.native_cycle_units() == 1 && bus.last_dma_cycle() == 0,
          "A rejected finish advances neither guest time nor DMA cursor");

  blofeld::DspBus unchanged_rate;
  reset(unchanged_rate);
  auto unchanged_clock = clock();
  prime(unchanged_clock, unchanged_rate);
  unchanged_clock.begin_step(unchanged_rate, {.pc = observed_pc, .omr = 0});
  unchanged_rate.pll().write(reset_command | pstp);
  unchanged_clock.finish_native_region(unchanged_rate, 1, inputs(1));
  require(!unchanged_clock.faulted() && unchanged_clock.elapsed_ticks() == 2 * reset_period &&
              unchanged_rate.last_dma_cycle() == 1,
          "A distinct retained command with the same period preserves existing finish semantics");
}

void test_shared_cache_keeps_both_clock_inputs() {
  blofeld::DspBus bus;
  reset(bus);
  const auto original = clock();
  constexpr std::uint64_t shorter_period = 28125;
  const blofeld::DspRunClock half_timebase(
      {.extal_hz = extal,
       .timebase_hz = timebase / 2,
       .switch_timing = blofeld::DspClockSwitch::functional_region_boundary});
  const blofeld::DspRunClock twice_extal(
      {.extal_hz = 2 * extal,
       .timebase_hz = timebase,
       .switch_timing = blofeld::DspClockSwitch::functional_region_boundary});
  require(original.next_step_ticks(bus, 1) == reset_period,
          "The first clock's literal reset period warms a shared lookup");
  require(half_timebase.next_step_ticks(bus, 1) == shorter_period,
          "The same PCTL and EXTAL with half the timebase has half the period");
  require(original.next_step_ticks(bus, 1) == reset_period,
          "Returning to the first owner cannot retain the other timebase's result");
  require(twice_extal.next_step_ticks(bus, 1) == shorter_period,
          "The same PCTL and timebase with twice EXTAL has half the period");
  require(original.next_step_ticks(bus, 1) == reset_period && original.elapsed_ticks() == 0 &&
              half_timebase.elapsed_ticks() == 0 && twice_extal.elapsed_ticks() == 0,
          "Interleaved independent previews retain their rates without advancing any model");
}

void test_ratio_cache_reads_every_current_bus() {
  blofeld::DspBus bus;
  reset(bus);
  const auto bypass = bus.pll().target_ratio();
  require(bypass.numerator == 1 && bypass.denominator == 1,
          "A literal bypass ratio warms the shared command lookup");
  blofeld::DspBus unknown;
  require(refuses([&] { static_cast<void>(unknown.pll().target_ratio()); }).contains("PINIT"),
          "A second owner's unknown reset must fail even with the same cached command");

  struct Case {
    unsigned control;
    unsigned numerator;
    unsigned denominator;
  };

  constexpr std::array cases{Case{.control = 0x04601e, .numerator = 15, .denominator = 2},
                             Case{.control = 0x04e01e, .numerator = 15, .denominator = 2},
                             Case{.control = 0x04e11e, .numerator = 15, .denominator = 4},
                             Case{.control = 0x04a11e, .numerator = 15, .denominator = 2},
                             Case{.control = 0x08a11e, .numerator = 15, .denominator = 4},
                             Case{.control = 0x08a10f, .numerator = 15, .denominator = 8},
                             Case{.control = 0x08810f, .numerator = 1, .denominator = 2}};
  for (const auto &item : cases) {
    bus.pll().write(item.control);
    const auto ratio = bus.pll().target_ratio();
    require(ratio.numerator == item.numerator && ratio.denominator == item.denominator,
            "Every PLL field retains its independently specified reduced ratio");
    const auto repeated = bus.pll().target_ratio();
    require(repeated.numerator == item.numerator && repeated.denominator == item.denominator,
            "A repeated command returns the same independently specified ratio");
  }
  reset(unknown);
  const auto other = unknown.pll().target_ratio();
  const auto retained = bus.pll().target_ratio();
  require(other.numerator == 1 && other.denominator == 1 && retained.numerator == 1 &&
              retained.denominator == 2,
          "Interleaved bus owners retain distinct bypass divisions");
}

}

int main() {
  try {
    test_unknown_reset_and_failed_periods();
    test_all_command_fields_and_restore();
    test_native_finish_detects_rate_changes();
    test_shared_cache_keeps_both_clock_inputs();
    test_ratio_cache_reads_every_current_bus();
    std::cout << "Clock cache contract: " << assertions << " assertions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
