// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
using namespace blofeld::test;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr unsigned pctl = 0x02e112;
constexpr unsigned epoch = 1000;
constexpr unsigned first_end = 1090;
constexpr unsigned command_end = 1135;
constexpr unsigned final_end = 1155;
constexpr unsigned absolute_jump = 0x0af080;

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal_hz,
                    .timebase_hz = timebase_hz,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates};
}

blofeld::DspScheduleInputs schedule_inputs(const std::uint64_t start = 0) {
  return {.epoch_ticks = start,
          .observation = blofeld::DspStepObservation::reference_completion_boundary};
}

void initialize(blofeld::DspMachine &machine, const DspProgram &program) {
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
}

void plan(const blofeld::DspMachine &machine, const unsigned units, const unsigned ticks) {
  const auto before = machine.cpu().debug_state();
  const auto clock = machine.run_clock();
  const auto preview = machine.plan_step();
  require(preview.before == before && preview.native_cycle_units == units &&
              preview.start_ticks == clock.elapsed_ticks() && preview.duration_ticks == ticks &&
              machine.cpu().debug_state() == before &&
              machine.run_clock().elapsed_ticks() == clock.elapsed_ticks() &&
              machine.run_clock().native_cycle_units() == clock.native_cycle_units(),
          "Nominal preview must match independent literal costs without CPU/clock side effects");
}

void test_latched_repeat_uses_executed_word_cost() {
  DspProgram program;
  constexpr unsigned destination = 0x100;
  program.immediate("r0", destination);
  program.immediate("x0", 7);
  program.emit("rep #3");
  const auto body = program.pc();
  program.emit("nop");
  // FM JMP ea, MMM=110/RRR=000; SDK assembler lacks a long absolute branch form.
  program.word(absolute_jump);
  program.word(body);
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  constexpr unsigned long_move_ticks = 90;
  plan(machine, 2, long_move_ticks);
  machine.cpu().step();
  machine.cpu().step();
  constexpr unsigned rep_ticks = 225;
  plan(machine, 5, rep_ticks);
  machine.cpu().step();
  constexpr unsigned nop_ticks = 45;
  plan(machine, 1, nop_ticks);
  machine.cpu().step();
  const auto frozen = machine.checkpoint();
  const auto replacement = fixture::assemble("move x0,p:(r0)");
  require(replacement.size() == 1, "Own P: MOVEM replacement is a single instruction word");
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    // Controlled ROM-cache mutation, not physical ROM programming.
    machine.bus().install_rom({.start = body, .words = replacement});
    for (unsigned repeated = 0; repeated < 2; ++repeated) {
      plan(machine, 1, nop_ticks);
      machine.cpu().step();
    }
    constexpr unsigned after_rep_units = 12;
    constexpr unsigned after_rep_ticks = 540;
    require(machine.run_clock().native_cycle_units() == after_rep_units &&
                machine.run_clock().elapsed_ticks() == after_rep_ticks,
            "Remaining latched NOPs cost one each even when backing P: holds a six-unit MOVEM");
    constexpr unsigned jump_ticks = 180;
    plan(machine, 4, jump_ticks); // FM A-4 JMP long absolute: 3 + lab1.
    machine.cpu().step();
    constexpr unsigned movem_ticks = 270;
    plan(machine, 6, movem_ticks); // FM A-7 register to P:(Rn): six.
    machine.cpu().step();
    constexpr unsigned completed_units = 22;
    constexpr unsigned completed_ticks = 990;
    require(machine.run_clock().native_cycle_units() == completed_units &&
                machine.run_clock().elapsed_ticks() == completed_ticks &&
                machine.bus().read({.address = destination}) == 7,
            "A later normal fetch sees the changed instruction, its independent cost and store");
  }
}

void test_self_modifying_store_keeps_its_executed_cost() {
  DspProgram program;
  constexpr unsigned destination = 0x100;
  program.immediate("r0", destination);
  program.immediate("x0", 0); // Independent NOP encoding stored by the later MOVEM.
  program.word(absolute_jump);
  program.word(destination);
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  const auto store = fixture::assemble("move x0,p:(r0)");
  require(store.size() == 1, "Own self-modifying P: MOVEM retains its single-word form");
  machine.bus().write({.address = destination, .value = store.front()});
  for (unsigned setup = 0; setup < 3; ++setup) { machine.cpu().step(); }
  constexpr unsigned store_ticks = 270;
  plan(machine, 6, store_ticks);
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    blofeld::DspScheduledRun runner(schedule_inputs(), machine);
    constexpr unsigned completed_ticks = 630;
    require(runner.next_completion_ticks(machine) == completed_ticks &&
                runner.advance_to(
                    machine, completed_ticks, blofeld::DspScheduleBoundary::through, 1) == 1 &&
                machine.run_clock().native_cycle_units() == 14 &&
                machine.run_clock().elapsed_ticks() == completed_ticks &&
                machine.bus().read({.address = destination}) == 0,
            "An executed six-unit store does not acquire its replacement NOP's one-unit cost");
  }
}

void test_exact_deadlines_ratio_switch_and_mid_step_restore() {
  DspProgram program;
  program.immediate("x0", pctl);
  program.emit("movep x0,x:<<$fffffd");
  program.emit("nop");
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  blofeld::DspScheduledRun runner(schedule_inputs(epoch), machine);
  constexpr unsigned before_first = first_end - 1;
  require(runner.advance_to(machine, before_first, blofeld::DspScheduleBoundary::through, 4) == 0 &&
              machine.run_clock().native_cycle_units() == 0,
          "A completion scheduler never executes an instruction whose end exceeds the horizon");
  require(runner.advance_to(machine, first_end, blofeld::DspScheduleBoundary::before, 4) == 0,
          "Exclusive ties let the external source supply its observation first");
  require(runner.advance_to(machine, first_end, blofeld::DspScheduleBoundary::through, 4) == 1,
          "Inclusive ties execute the exact completion once");
  constexpr unsigned mid_command = command_end - 1;
  require(runner.advance_to(machine, mid_command, blofeld::DspScheduleBoundary::through, 4) == 0,
          "A pending PCTL command still uses the previously declared reference rate");
  const auto cpu = machine.checkpoint();
  const auto pending = runner.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(cpu);
    runner.restore(pending, machine);
    std::vector<std::uint64_t> completions;
    const auto observe = [&](const blofeld::DspScheduledStep &step) {
      completions.push_back(step.completion_ticks);
      static_cast<void>(fixture::rejects([&] { static_cast<void>(runner.checkpoint(machine)); },
                                         "An active schedule cannot become a trusted snapshot"));
    };
    require(runner.advance_to(
                machine, final_end, blofeld::DspScheduleBoundary::through, 4, observe) == 3,
            "Pending command and two following NOPs consume three real native steps");
    constexpr unsigned first_fast_end = 1145;
    constexpr unsigned final_ticks = 155;
    require(completions == std::vector<std::uint64_t>{command_end, first_fast_end, final_end} &&
                machine.run_clock().native_cycle_units() == 5 &&
                machine.run_clock().elapsed_ticks() == final_ticks &&
                runner.horizon_ticks() == final_end && runner.completed_steps() == 4,
            "Snapshot retains unfinished deadline, old/new PLL rate and exact completion phase");
    require(runner.advance_to(machine, final_end, blofeld::DspScheduleBoundary::through, 0) == 0,
            "A repeated already-completed horizon consumes no extra clock or instruction");
  }
}

void test_budget_failure_and_coordinated_restore() {
  DspProgram program;
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  blofeld::DspScheduledRun runner(schedule_inputs(), machine);
  const auto cpu = machine.checkpoint();
  const auto schedule = runner.checkpoint(machine);
  constexpr unsigned end = 135;
  const auto error = fixture::rejects(
      [&] { runner.advance_to(machine, end, blofeld::DspScheduleBoundary::through, 1); },
      "Insufficient execution budget cannot claim the requested horizon");
  require(runner.faulted() && machine.run_clock().native_cycle_units() == 1 &&
              error.contains("budget") && error.contains("pc=") && error.contains("horizon=") &&
              error.contains("attempted-completion=none"),
          "Budget failure retains only the completed native prefix and explicit context");
  static_cast<void>(fixture::rejects([&] { runner.restore(schedule, machine); },
                                     "Unilateral scheduler restore must reject changed CPU time"));
  machine.restore(cpu);
  runner.restore(schedule, machine);
  require(runner.advance_to(machine, end, blofeld::DspScheduleBoundary::through, 3) == 3,
          "Trusted coordinated restore recovers both time owners without skipped instructions");
}

void test_changed_pending_instruction_and_callback() {
  DspProgram program;
  program.emit("nop");
  program.emit("nop");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  blofeld::DspScheduledRun runner(schedule_inputs(), machine);
  runner.advance_to(machine, 1, blofeld::DspScheduleBoundary::through, 1);
  const auto cpu = machine.checkpoint();
  const auto pending = runner.checkpoint(machine);
  const auto replacement = fixture::assemble("move #>$123456,x0");
  machine.bus().install_rom({.start = boot, .words = replacement});
  constexpr unsigned nop_end = 45;
  const auto error = fixture::rejects(
      [&] { runner.advance_to(machine, nop_end, blofeld::DspScheduleBoundary::through, 1); },
      "Changed duration cannot execute using an older completion deadline");
  require(runner.faulted() && machine.run_clock().native_cycle_units() == 0 &&
              error.contains("changed before"),
          "Pending-cost mismatch rejects before executing or advancing the CPU");
  machine.restore(cpu);
  runner.restore(pending, machine);
  static_cast<void>(fixture::rejects(
      [&] {
        runner.advance_to(
            machine, nop_end, blofeld::DspScheduleBoundary::through, 1, [&](const auto & /*step*/) {
              machine.cpu().step();
            });
      },
      "A callback must not secretly advance the native owner"));
  require(runner.faulted() && machine.run_clock().native_cycle_units() == 1,
          "Callback mutation is detected before the scheduler executes another instruction");
}

void test_unknown_program_and_epoch_overflow() {
  blofeld::DspMachine machine(contract());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  blofeld::DspScheduledRun runner(schedule_inputs(), machine);
  const auto error = fixture::rejects(
      [&] { runner.advance_to(machine, epoch, blofeld::DspScheduleBoundary::through, 4); },
      "Missing original code must fail during strict cost preparation");
  require(runner.faulted() && machine.run_clock().native_cycle_units() == 0 &&
              error.contains("0xff0000") && error.contains("attempted-completion=none"),
          "Unknown code never becomes a speculative NOP or guessed duration");
  DspProgram program;
  program.emit("nop");
  initialize(machine, program);
  constexpr auto near_end = std::numeric_limits<std::uint64_t>::max() - 30;
  blofeld::DspScheduledRun exhausted(schedule_inputs(near_end), machine);
  static_cast<void>(fixture::rejects(
      [&] {
        exhausted.advance_to(machine,
                             std::numeric_limits<std::uint64_t>::max(),
                             blofeld::DspScheduleBoundary::through,
                             1);
      },
      "A completion beyond the declared epoch must fail without arithmetic wrap"));
  require(exhausted.faulted() && machine.run_clock().native_cycle_units() == 0,
          "Exhausted absolute time rejects before any native step");
}

void test_operand_fault_deadline_and_native_recovery() {
  DspProgram program;
  constexpr unsigned reserved = 0x2000;
  program.immediate("r0", reserved);
  program.immediate("x0", 7);
  program.emit("move x0,y:(r0)");
  blofeld::DspMachine machine(contract());
  initialize(machine, program);
  blofeld::DspScheduledRun runner(schedule_inputs(epoch), machine);
  constexpr unsigned setup_end = 1180;
  constexpr unsigned store_end = 1225; // Two long immediates: 2+2; MOVE Y:(Rn): 1.
  constexpr unsigned setup_ticks = setup_end - epoch;
  constexpr unsigned store_ticks = store_end - setup_end;
  constexpr unsigned horizon = store_end + 17;
  require(runner.advance_to(machine, setup_end, blofeld::DspScheduleBoundary::through, 2) == 2,
          "Two independent two-unit immediate expectations precede the operand fault");
  plan(machine, 1, store_ticks);
  const auto frozen = machine.checkpoint();
  const auto pending = runner.checkpoint(machine);
  std::string expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    runner.restore(pending, machine);
    const auto error = fixture::rejects(
        [&] { runner.advance_to(machine, horizon, blofeld::DspScheduleBoundary::through, 2); },
        "An actual unqualified operand must fail at its scheduled completion");
    require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                error.contains("horizon=1242") && error.contains("attempted-completion=1225") &&
                error.contains("completed-reference-ticks=180") &&
                error.contains("completed-native-cycle-units=4") && runner.completed_steps() == 2 &&
                runner.horizon_ticks() == setup_end && runner.faulted() &&
                machine.timing_faulted() && machine.run_clock().elapsed_ticks() == setup_ticks,
            "Failed duration is diagnosed separately from the horizon and completed native time");
    static_cast<void>(
        fixture::rejects([&] { static_cast<void>(machine.checkpoint()); },
                         "A faulted operand cannot become a trusted native snapshot"));
    if (replay == 0) { expected = error; }
    require(error == expected, "Coordinated recovery reproduces the actual operand fault exactly");
  }
}
}

int main() {
  try {
    test_latched_repeat_uses_executed_word_cost();
    test_self_modifying_store_keeps_its_executed_cost();
    test_exact_deadlines_ratio_switch_and_mid_step_restore();
    test_budget_failure_and_coordinated_restore();
    test_changed_pending_instruction_and_callback();
    test_unknown_program_and_epoch_overflow();
    test_operand_fault_deadline_and_native_recovery();
    std::cout << "DSP nominal previews, exact completion bounds, REP cost and restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
