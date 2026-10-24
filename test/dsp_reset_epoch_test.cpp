// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld::test;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned reset_pctl = 0x04401d;
constexpr unsigned stage_pctl = 0x02e112;
constexpr unsigned count0 = 0xffff8c;
constexpr unsigned load0 = 0xffff8e;
constexpr unsigned compare0 = 0xffff8d;
constexpr unsigned csr0 = 0xffff8f;
constexpr unsigned schedule_epoch = 1000;
constexpr unsigned initial_prefix_ticks = 370;
constexpr unsigned reset_prefix_ticks = 135;
constexpr unsigned long_move_ticks = 90;
constexpr unsigned nop_ticks = 45;
constexpr unsigned invalid_policy = 255;
constexpr unsigned partial_horizon = 1020;
constexpr unsigned first_deadline = 1045;
constexpr unsigned second_deadline = 1090;

blofeld::DspRunContract contract(const bool resets = true) {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates,
          .interrupts = blofeld::DspInterruptTiming::native_vector_estimates,
          .resets = resets ? blofeld::DspResetTiming::reference_instruction_boundary
                           : blofeld::DspResetTiming::unqualified};
}

void initialize(blofeld::DspMachine &machine, const DspProgram &program) {
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
}

void counts(const blofeld::DspMachine &machine,
            const unsigned total,
            const unsigned local,
            const unsigned ticks,
            const unsigned edges) {
  const auto &clock = machine.run_clock();
  require(!machine.timing_faulted() && clock.native_cycle_units() == total &&
              clock.epoch_native_cycle_units() == local &&
              machine.cpu().debug_state().native_cycles == local &&
              clock.elapsed_ticks() == ticks && clock.timer_clk2_edges() == edges,
          "Reset epochs must retain literal lifetime time/units and their own native CLK2 phase");
}

void timer(blofeld::DspMachine &machine, const unsigned value) {
  auto &timers = machine.bus().timers();
  timers.write({.address = csr0, .value = 0});
  timers.write({.address = load0, .value = value});
  timers.write({.address = compare0, .value = word_mask});
  timers.write({.address = csr0, .value = 7});
}

void test_time_memory_pll_phase_and_restore() {
  DspProgram program;
  program.emit("nop");
  program.immediate("x0", fixture::cookie);
  program.emit("move x0,x:>$100");
  program.immediate("x0", stage_pctl);
  program.emit("movep x0,x:<<$fffffd");
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  timer(machine, 7);
  fixture::run(machine, 6); // 1+2+2+2+1 old-rate units, then one new-rate unit.
  counts(machine, 9, 9, initial_prefix_ticks, 4);
  require(machine.bus().timers().read(count0) == 10, "Four real CLK2 edges load7 then count8/9/10");
  const auto before_reset = machine.checkpoint();
  const auto initial_epoch = machine.run_clock().native_epoch();

  struct Expected {
    unsigned total;
    unsigned ticks;
    unsigned edges;
    unsigned load;
  };

  constexpr std::array expected{Expected{.total = 12, .ticks = 505, .edges = 5, .load = 20},
                                Expected{.total = 15, .ticks = 640, .edges = 6, .load = 21}};
  for (const auto &value : expected) {
    const auto previous_epoch = machine.run_clock().native_epoch();
    machine.cpu().reset_spi_boot();
    counts(machine, value.total - 3, 0, value.ticks - reset_prefix_ticks, value.edges - 1);
    require(machine.run_clock().native_epoch() == previous_epoch + 1 &&
                machine.cpu().state().pc == boot && machine.cpu().state().omr == reset_omr &&
                machine.bus().pll().read() == reset_pctl &&
                machine.bus().read({.space = blofeld::DspSpace::x, .address = fixture::data}) ==
                    fixture::cookie &&
                machine.bus().read({.address = boot}) == 0 &&
                machine.bus().timers().read(csr0, 1) == 0,
            "Native reset retains RAM/ROM, resets the PLL/peripherals and selects SPI boot");
    // DSP56371UM Appendix C: TCR resets to zero; TCPR is undefined.
    require(machine.bus().timers().read(count0) == 0, "Reset clears the actual timer counter");
    static_cast<void>(
        fixture::rejects([&] { static_cast<void>(machine.bus().timers().read(compare0)); },
                         "Reset must not invent a timer compare value"));
    timer(machine, value.load);
    machine.cpu().step(); // New-epoch NOP has no CLK2 edge despite the old odd residue.
    counts(machine, value.total - 2, 1, value.ticks - long_move_ticks, value.edges - 1);
    require(machine.bus().timers().read(count0) == 0,
            "An old epoch's odd residue cannot load the timer in the new epoch");
    const auto partial = machine.checkpoint();
    machine.cpu().step();
    const auto completed = machine.cpu().debug_state();
    counts(machine, value.total, 3, value.ticks, value.edges);
    require(machine.bus().timers().read(count0) == value.load,
            "The first new CLK2 edge loads the independently declared counter");
    machine.restore(partial);
    machine.cpu().step();
    counts(machine, value.total, 3, value.ticks, value.edges);
    require(machine.cpu().debug_state() == completed &&
                machine.bus().timers().read(count0) == value.load,
            "A partial new epoch replays with matching native state and retained timer phase");
  }
  machine.restore(before_reset);
  counts(machine, 9, 9, initial_prefix_ticks, 4);
  require(machine.run_clock().native_epoch() == initial_epoch &&
              machine.bus().pll().read() == stage_pctl,
          "Trusted restore recovers the pre-reset epoch and PLL together");
}

void test_default_refusal_and_fault_recovery() {
  DspProgram program;
  program.emit("nop");
  program.emit("nop");
  blofeld::DspMachine strict(contract(false));
  initialize(strict, program);
  strict.cpu().step();
  const auto before = strict.cpu().debug_state();
  const auto epoch = strict.run_clock().native_epoch();
  const auto error = fixture::rejects([&] { strict.cpu().reset_spi_boot(); },
                                      "Live reset must require an explicit timing contract");
  require(error.contains("explicit reference reset timing") && error.contains("pc=0xff0001") &&
              error.contains("omr=0x000305") && strict.cpu().debug_state() == before &&
              strict.run_clock().native_epoch() == epoch,
          "Default refusal retains native state, epoch and execution context");
  counts(strict, 1, 1, nop_ticks, 0);
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  machine.cpu().step();
  const auto healthy = machine.checkpoint();
  machine.bus().timers().write({.address = csr0, .value = 1});
  static_cast<void>(fixture::rejects([&] { machine.cpu().step(); }, "Unknown timer must fail"));
  const auto fault_epoch = machine.run_clock().native_epoch();
  const auto fault = fixture::rejects([&] { machine.cpu().reset_spi_boot(); },
                                      "Reset cannot hide a partially timed failed instruction");
  require(machine.timing_faulted() && fault.contains("coordinated recovery") &&
              machine.run_clock().native_epoch() == fault_epoch &&
              machine.run_clock().elapsed_ticks() == nop_ticks,
          "Failed timing remains quarantined with its completed prefix");
  machine.restore(healthy);
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  counts(machine, 2, 1, long_move_ticks, 0);
  auto invalid = contract();
  invalid.resets = static_cast<blofeld::DspResetTiming>(invalid_policy);
  static_cast<void>(fixture::rejects([&] { const blofeld::DspMachine rejected(invalid); },
                                     "Unknown reset policies cannot select successful behavior"));
}

blofeld::DspScheduleInputs schedule(const blofeld::DspIrqAdmission admission) {
  return {.epoch_ticks = schedule_epoch,
          .observation = blofeld::DspStepObservation::reference_completion_boundary,
          .irq_admission = admission};
}

void test_stale_schedules_and_rearm() {
  for (const auto admission : {blofeld::DspIrqAdmission::reject_changes,
                               blofeld::DspIrqAdmission::reference_prepare_boundary}) {
    DspProgram program;
    program.emit("nop");
    program.emit("nop");
    blofeld::DspMachine machine(contract());
    initialize(machine, program);
    const auto owner = machine.checkpoint();
    blofeld::DspScheduledRun runner(schedule(admission), machine);
    const auto saved = runner.checkpoint(machine);
    require(runner.advance_to(machine, partial_horizon, blofeld::DspScheduleBoundary::through, 1) ==
                0,
            "A prepared instruction before its deadline must remain unexecuted");
    machine.cpu().reset_spi_boot();
    static_cast<void>(fixture::rejects(
        [&] {
          runner.advance_to(machine, first_deadline, blofeld::DspScheduleBoundary::through, 1);
        },
        "Even a zero-time reset invalidates an otherwise identical prepared instruction"));
    require(runner.faulted(), "A stale reset epoch quarantines its schedule before dispatch");
    counts(machine, 0, 0, 0, 0);
    machine.restore(owner);
    runner.restore(saved, machine);
    require(runner.advance_to(machine, first_deadline, blofeld::DspScheduleBoundary::through, 1) ==
                1,
            "Joint recovery executes the original pre-reset native instruction");
    machine.cpu().reset_spi_boot();
    static_cast<void>(
        fixture::rejects([&] { static_cast<void>(runner.next_completion_ticks(machine)); },
                         "A live reset also invalidates a schedule with no pending instruction"));
    blofeld::DspScheduledRun restarted(schedule(admission), machine);
    require(restarted.next_completion_ticks(machine) == second_deadline &&
                restarted.advance_to(
                    machine, second_deadline, blofeld::DspScheduleBoundary::through, 1) == 1,
            "Explicit rearming at a completed native boundary retains the absolute deadline");
    counts(machine, 2, 1, long_move_ticks, 0);
  }
}

void test_observer_reset_is_visible() {
  DspProgram program;
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  blofeld::DspScheduledRun runner(schedule(blofeld::DspIrqAdmission::reject_changes), machine);
  const auto error = fixture::rejects(
      [&] {
        runner.advance_to(
            machine,
            first_deadline,
            blofeld::DspScheduleBoundary::through,
            1,
            {},
            {.observe = [&](const auto & /*step*/) { machine.cpu().reset_spi_boot(); }});
      },
      "A completion observer cannot silently reset its native owner");
  require(runner.faulted() && error.contains("post-completion observation") &&
              error.contains("completed-steps=1"),
          "Observer reset rejection preserves the actually completed prefix");
  counts(machine, 1, 0, nop_ticks, 0);
}
}

int main() {
  try {
    test_time_memory_pll_phase_and_restore();
    test_default_refusal_and_fault_recovery();
    test_stale_schedules_and_rearm();
    test_observer_reset_is_visible();
    std::cout << "Native reset epochs retain reference time, strict failures and stale-schedule "
                 "guards.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
