// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::require;
constexpr unsigned extal = 12000000, timebase = 540000000, period = 45;
constexpr unsigned address = 0x100, data = 0x123456, reset_omr = 0x305;
constexpr unsigned reset_status = 0xc00300, recovery_cycles = 3, reset_ticks = 225;

DspRunContract contract(const DspRunTiming timing) {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = timing,
          .resets = DspResetTiming::reference_extal_hold};
}

template <typename Operation> void held_rejection(Operation operation) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains("hardware reset"),
            "An instruction or plan during held reset names its actual blocked context");
    return;
  }
  throw Fault("Native instruction or plan escaped the held hardware reset");
}

void held_reset(DspMachine &machine, const DspRunTiming timing, const unsigned stalls) {
  const auto start = machine.run_clock().elapsed_ticks();
  const auto edges = (3 + stalls) / 2;
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = recovery_cycles});
  require(machine.cpu().state().pc == blofeld::test::boot &&
              machine.cpu().state().omr == reset_omr && machine.cpu().state().sr == reset_status &&
              machine.run_clock().native_cycle_units() == 3 &&
              machine.run_clock().interlock_cycle_units() == stalls &&
              machine.run_clock().epoch_native_cycle_units() == 0 &&
              machine.run_clock().epoch_interlock_cycle_units() == 0,
          "Actual native reset preserves lifetime clocks and clears every dependency epoch");
  held_rejection([&] { machine.cpu().step(); });
  held_rejection([&] { static_cast<void>(machine.plan_step()); });
  require(!machine.timing_faulted() && machine.run_clock().elapsed_ticks() == start,
          "Rejected held operations perform no native work or reference-time advancement");
  require(machine.advance_hardware_reset_ticks(period + 1) == period + 1,
          "The asserted reset owns its actual partial EXTAL period");
  const auto asserted = machine.checkpoint();
  DspMachine fresh(contract(timing));
  fresh.restore(asserted);
  require(fresh.advance_hardware_reset_ticks(period - 1) == period - 1,
          "Fresh reset owner completes exactly two asserted EXTAL periods");
  fresh.release_hardware_reset();
  constexpr auto partial_recovery = (std::uint64_t{recovery_cycles} * period) - 1;
  require(fresh.advance_hardware_reset_ticks(partial_recovery) == partial_recovery &&
              fresh.hardware_reset_active(),
          "Recovery cannot release native execution before its last supplied tick");
  held_rejection([&] { fresh.cpu().step(); });
  const auto recovering = fresh.checkpoint();
  machine.restore(recovering);
  require(machine.advance_hardware_reset_ticks(1) == 1 && !machine.hardware_reset_active() &&
              machine.run_clock().hardware_reset_ticks() == reset_ticks &&
              machine.run_clock().elapsed_ticks() == start + reset_ticks &&
              machine.run_clock().native_cycle_units() == 3 &&
              machine.run_clock().interlock_cycle_units() == stalls &&
              machine.run_clock().timer_clk2_edges() == edges,
          "Fresh recovery retains real reset time without inventing CPU or timer clocks");
}

void native_pair(DspMachine &machine, const unsigned stalls, const std::uint64_t run) {
  machine.cpu().step(); // Independent long-immediate producer: two native units.
  const auto before = machine.cpu().debug_state();
  const auto plan = machine.plan_step();
  require(before.cpu.r[0] == address && plan.native_cycle_units == 1 &&
              plan.interlock_cycle_units == stalls &&
              plan.duration_ticks == (std::uint64_t{1} + stalls) * period &&
              machine.cpu().debug_state() == before,
          "Separate nominal and initial-map contracts retain their literal 0/3/2 dependency cost");
  machine.cpu().step();
  const auto &clock = machine.run_clock();
  const auto instruction_ticks = (run * (3 + stalls)) * period;
  const auto hardware_ticks = (run - 1) * reset_ticks;
  require(
      (machine.cpu().state().x & blofeld::test::word_mask) == data &&
          clock.native_cycle_units() == run * 3 && clock.interlock_cycle_units() == run * stalls &&
          clock.elapsed_cycle_units() == run * (3 + stalls) &&
          clock.elapsed_ticks() == instruction_ticks + hardware_ticks &&
          clock.epoch_native_cycle_units() == 3 && clock.epoch_interlock_cycle_units() == stalls &&
          clock.timer_clk2_edges() == run * ((3 + stalls) / 2),
      "Native load, inserted dependency clocks and fresh timer epoch advance exactly once");
}

void exercise(const DspRunTiming timing, const unsigned stalls) {
  DspMachine machine(contract(timing));
  DspProgram code;
  code.immediate("r0", address);
  code.emit("move x:(r0),x0");
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = data});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  native_pair(machine, stalls, 1);
  held_reset(machine, timing, stalls);
  native_pair(machine, stalls, 2);
}
}

int main() {
  try {
    // FM A-2 nominal MOVE delay versus the independently qualified initial-map
    // long-immediate window: retain both explicit contracts, never sum them.
    exercise(DspRunTiming::native_internal_estimates, 0);
    exercise(DspRunTiming::native_internal_agu_interlocks, 3);
    exercise(DspRunTiming::native_interlock_estimates, 2);
    std::cout << "Native DSP reset and separate dependency-clock contracts pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
