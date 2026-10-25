// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal = 12000000, timebase = 540000000, extal_ticks = 45;
constexpr unsigned mode_mask = 15, spi_mode = 5, byte_values = 256;
constexpr unsigned reset_omr = 0x305; // FM figure 5-3/table 5-2: CDP1/CDP0 and mode-five pins.
constexpr unsigned reset_status = 0xc00300, reset_pctl = 0x04401d, stage_pctl = 0x02e112;
constexpr unsigned prefix_units = 5, prefix_ticks = 225, prefix_clk2 = 2;
constexpr unsigned resume_units = 3, resume_ticks = 135, final_clk2 = 3;
constexpr unsigned csr0 = 0xffff8f, counter0 = 0xffff8c, load0 = 0xffff8e;
constexpr unsigned epoch = 1000, excess = 99, port_mask = 0xfff, compare0 = 0xffff8d;
constexpr unsigned iprc = 0xffffff;

DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates,
          .interrupts = DspInterruptTiming::native_vector_estimates,
          .resets = DspResetTiming::reference_extal_hold};
}

DspHardwareResetInputs pins(const unsigned recovery = 3) {
  return {.mode_known = mode_mask,
          .mode_levels = spi_mode,
          .pinit = false,
          .recovery_cycles = recovery};
}

DspScheduleInputs scheduled() {
  return {.epoch_ticks = epoch, .observation = DspStepObservation::reference_completion_boundary};
}

void reset_registers(const DspMachine &machine) {
  const auto state = machine.cpu().debug_state();
  require(state.cpu.pc == boot && state.cpu.omr == reset_omr && state.cpu.sr == reset_status &&
              state.cpu.sp == 0 && state.sc == 0 && state.cpu.vba == 0 &&
              state.native_cycles == 0 && state.native_instructions == 0,
          "Hardware reset has independent native PC/mode/SR/stack/counter expectations");
  for (const auto value : state.cpu.m) {
    require(value == word_mask, "Hardware reset initializes every native modulo register");
  }
}

void initialize(DspMachine &machine) {
  DspProgram program;
  program.emit("nop");
  program.immediate("x0", fixture::cookie);
  program.emit("move x0,x:>$100");
  program.emit("nop");
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned n = 0; n < 3; ++n) { machine.cpu().step(); }
  require(machine.run_clock().native_cycle_units() == prefix_units &&
              machine.run_clock().elapsed_ticks() == prefix_ticks &&
              machine.run_clock().timer_clk2_edges() == prefix_clk2,
          "Own NOP/long-immediate/extended-MOVE prefix has literal1+2+2 unit costs");
  machine.bus().pll().write(stage_pctl);
  auto &port = machine.bus().gpio(DspPort::c);
  port.write(
      {.reg = DspGpioRegister::data, .value = fixture::cookie & port_mask, .mask = port_mask});
  port.write({.reg = DspGpioRegister::direction, .value = port_mask, .mask = port_mask});
  auto &timer = machine.bus().timers();
  timer.write({.address = load0, .value = fixture::cookie});
  timer.write({.address = compare0, .value = word_mask});
  timer.write({.address = csr0, .value = 7});
  machine.bus().observe_irqa(false); // The explicit mode5 observation replaces the prior pin level.
}

void blocked(DspMachine &machine) {
  const auto state = machine.cpu().debug_state();
  const auto ticks = machine.run_clock().elapsed_ticks();
  static_cast<void>(fixture::rejects([&] { machine.cpu().step(); }, "Held DSP executed a step"));
  static_cast<void>(fixture::rejects([&] { static_cast<void>(machine.plan_step()); },
                                     "Held DSP previewed an instruction"));
  static_cast<void>(fixture::rejects([&] { machine.cpu().reset_spi_boot(); },
                                     "Held DSP admitted a second native reset"));
  static_cast<void>(fixture::rejects([&] { const DspScheduledRun invalid(scheduled(), machine); },
                                     "Held DSP admitted a new instruction scheduler"));
  require(machine.cpu().debug_state() == state && machine.run_clock().elapsed_ticks() == ticks &&
              !machine.timing_faulted(),
          "Held instruction/plan/reset/scheduler refusals preserve native state and clocks");
}

void retained(DspMachine &machine) {
  require(machine.bus().read({.space = DspSpace::x, .address = fixture::data}) == fixture::cookie &&
              machine.bus().peek_program(boot) == 0 &&
              machine.bus().gpio(DspPort::c).outputs().mask == 0 &&
              machine.bus().timers().read(csr0, 1) == 0 &&
              machine.bus().timers().read(counter0) == 0 &&
              machine.bus().pll().read() == reset_pctl,
          "Hardware reset retains native-written RAM/ROM, resets GPIO/TIM and samples PINIT-low");
  static_cast<void>(fixture::rejects([&] { static_cast<void>(machine.bus().timers().read(csr0)); },
                                     "Reset invented an unobserved external timer DI level"));
  auto copy = machine.bus();
  copy.write({.space = DspSpace::x, .address = iprc, .value = 1});
  require(!copy.interrupt_request(0),
          "Observed mode5 MODA-high remains the known inactive IRQA input after reset");
}

DspDebugState finish(DspMachine &machine, const unsigned assertion, const unsigned recovery) {
  require(machine.advance_hardware_reset(excess) == recovery - 1 &&
              !machine.hardware_reset_active(),
          "Recovery consumes only the remaining qualified EXTAL periods, never an extra tick");
  reset_registers(machine);
  const auto hold_ticks = (assertion + recovery) * extal_ticks;
  const auto &clock = machine.run_clock();
  require(clock.elapsed_ticks() == prefix_ticks + hold_ticks &&
              clock.hardware_reset_ticks() == hold_ticks &&
              clock.native_cycle_units() == prefix_units && clock.epoch_native_cycle_units() == 0 &&
              clock.timer_clk2_edges() == prefix_clk2,
          "Asserted/recovery time advances no native unit, timer CLK2 edge or instruction");
  retained(machine);
  DspScheduledRun resumed(scheduled(), machine);
  const auto first = epoch + prefix_ticks + hold_ticks + extal_ticks;
  require(resumed.next_completion_ticks(machine) == first &&
              resumed.advance_to(machine, first, DspScheduleBoundary::through, 1) == 1 &&
              resumed.advance_to(
                  machine, first + (2 * extal_ticks), DspScheduleBoundary::through, 1) == 1,
          "A fresh schedule resumes native NOP/MOVE at absolute deadlines including reset time");
  require(clock.elapsed_ticks() == prefix_ticks + hold_ticks + resume_ticks &&
              clock.native_cycle_units() == prefix_units + resume_units &&
              clock.epoch_native_cycle_units() == resume_units &&
              clock.timer_clk2_edges() == final_clk2 && clock.native_epoch() == 2,
          "Native reset restarts CLK2 odd-phase accounting without losing lifetime units/time");
  return machine.cpu().debug_state();
}

void test_hold_and_replay(const unsigned assertion, const unsigned recovery) {
  DspMachine machine(contract());
  initialize(machine);
  DspScheduledRun stale(scheduled(), machine);
  require(stale.advance_to(machine, epoch + prefix_ticks + 1, DspScheduleBoundary::before, 1) == 0,
          "The pre-reset scheduler retains its unexecuted pending instruction");
  machine.begin_hardware_reset(pins(recovery));
  reset_registers(machine);
  retained(machine);
  blocked(machine);
  require(machine.advance_hardware_reset(assertion) == assertion,
          "Every explicitly observed asserted EXTAL period is retained");
  const auto asserted = machine.checkpoint();
  static_cast<void>(fixture::rejects(
      [&] {
        static_cast<void>(
            stale.advance_to(machine, epoch + prefix_ticks + 2, DspScheduleBoundary::through, 1));
      },
      "Pre-reset scheduler survived native reset epoch"));
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset(1) == 1, "One real recovery period precedes the cut");
  blocked(machine);
  const auto cut = machine.checkpoint();
  const auto after = finish(machine, assertion, recovery);
  const auto elapsed = machine.run_clock().elapsed_ticks();
  const auto reset_state = machine.run_clock().hardware_reset_state();
  machine.restore(cut);
  require(machine.hardware_reset_active(), "Trusted recovery restores the execution hold");
  require(finish(machine, assertion, recovery) == after &&
              machine.run_clock().elapsed_ticks() == elapsed &&
              machine.run_clock().hardware_reset_state() == reset_state,
          "Mid-recovery restore replays complete native state and literal reset/clock ledgers");
  machine.restore(asserted);
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset(1) == 1 && finish(machine, assertion, recovery) == after,
          "A held-reset checkpoint replays release and actual native boot execution");
}

void test_input_and_failure_guards() {
  DspMachine machine(contract());
  initialize(machine);
  const auto before = machine.cpu().debug_state();
  const auto tick = machine.run_clock().elapsed_ticks();
  for (unsigned index = 0; index < 5; ++index) {
    auto input = pins();
    if (index == 0) { input.mode_known = 0; }
    if (index == 1) { input.mode_levels = 4; }
    if (index == 2) { input.pinit.reset(); }
    if (index == 3) { input.pinit = true; }
    if (index == 4) { input.recovery_cycles = 1; }
    static_cast<void>(fixture::rejects([&] { machine.begin_hardware_reset(input); },
                                       "Unqualified reset inputs were accepted"));
    require(machine.cpu().debug_state() == before && machine.run_clock().elapsed_ticks() == tick &&
                machine.bus().pll().read() == stage_pctl && !machine.timing_faulted(),
            "Unknown mode/PINIT, PLL lock and missing recovery reject before any reset mutation");
  }
  machine.begin_hardware_reset(pins());
  const auto saved = machine.checkpoint();
  for (const unsigned kind : {0U, 1U, 2U}) {
    machine.restore(saved);
    const auto state = machine.cpu().debug_state();
    static_cast<void>(fixture::rejects(
        [&] {
          if (kind == 0) { machine.release_hardware_reset(); }
          if (kind == 1) { static_cast<void>(machine.advance_hardware_reset(0)); }
          if (kind == 2) {
            static_cast<void>(
                machine.advance_hardware_reset(std::numeric_limits<std::uint64_t>::max()));
          }
        },
        "Invalid/short/exhausted reset did not fail"));
    require(machine.timing_faulted() && machine.cpu().debug_state() == state &&
                machine.run_clock().elapsed_ticks() == tick,
            "Failed reset periods retain their prefix and quarantine the owner");
  }
  machine.restore(saved);
  require(machine.advance_hardware_reset(1) == 1, "One asserted period remains too short");
  static_cast<void>(
      fixture::rejects([&] { machine.release_hardware_reset(); }, "One-period reset was accepted"));
  require(machine.timing_faulted(), "A short actual reset release requires trusted recovery");
  auto unsupported = contract();
  unsupported.resets = DspResetTiming::reference_instruction_boundary;
  DspMachine ordinary(unsupported);
  static_cast<void>(fixture::rejects([&] { ordinary.begin_hardware_reset(pins()); },
                                     "Legacy zero-duration contract accepted a reset hold"));
}

void test_all_reset_ccr_values() {
  for (unsigned flags = 0; flags < byte_values; ++flags) {
    DspBus bus;
    DspProgram program;
    program.immediate("sr", reset_status | flags);
    program.install(bus);
    DspCore cpu(bus);
    cpu.step();
    require(cpu.state().sr == (reset_status | flags), "Own native seed writes every CCR bit");
    cpu.reset_spi_boot();
    require(cpu.state().sr == reset_status,
            "DSP56300FM2.3.3 clears all eight CCR bits on actual native hardware reset");
  }
}
}

int main() {
  try {
    test_all_reset_ccr_values();
    for (const unsigned assertion : {2U, 5U}) {
      for (const unsigned recovery : {2U, 3U, 7U}) { test_hold_and_replay(assertion, recovery); }
    }
    test_input_and_failure_guards();
    std::cout << "Verified256 native CCR resets,6 explicit EXTAL holds,12 trusted suffix replays, "
                 "reset input/failure guards and absolute native schedule rearming.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
