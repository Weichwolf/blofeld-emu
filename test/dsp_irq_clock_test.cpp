// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld::test;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned initial_period = 45;
constexpr unsigned pll_period = 10;
constexpr unsigned stage_pctl = 0x02e112;
constexpr unsigned count0 = 0xffff8c;
constexpr unsigned timer_start = 17;
constexpr unsigned compare_flag = 0x200000;
constexpr unsigned higher_priorities = 0x300;
constexpr unsigned relocated_base = 0x500;
// FM Rev5 13-113: short immediates in X0/Y0 occupy bits 23..16.
constexpr unsigned fractional_one = 0x010000;
constexpr unsigned fractional_two = 0x020000;
// FM Rev5 13-89 JSR ea, MMMRRR=110000 (absolute extension).
constexpr unsigned absolute_jsr = 0x0bf080;

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates,
          .interrupts = blofeld::DspInterruptTiming::native_vector_estimates};
}

void vector(blofeld::DspMachine &machine,
            const unsigned address,
            const char *first,
            const char *second = "nop") {
  auto words = fixture::assemble(first);
  if (words.size() == 1) {
    const auto suffix = fixture::assemble(second);
    words.insert(words.end(), suffix.begin(), suffix.end());
  }
  require(words.size() == 2, "Own interrupt diagnostic supplies exactly two vector words");
  fixture::ram(machine.bus(), address, words);
}

void timer(blofeld::DspMachine &machine, const unsigned csr = fixture::csr0) {
  machine.bus().timers().write({.address = csr - 1, .value = timer_start});
  machine.bus().timers().write({.address = csr - 2, .value = timer_start});
  machine.bus().timers().write({.address = csr, .value = 7});
}

unsigned initialize(blofeld::DspMachine &machine,
                    const bool odd = false,
                    const unsigned base = 0,
                    const unsigned r0 = fixture::handler) {
  DspProgram program;
  program.immediate("x0", 0);
  program.immediate("y0", 0);
  program.immediate("r0", r0);
  program.immediate("vba", base);
  if (odd) { program.emit("nop"); }
  program.immediate("sr", fixture::ordinary_status);
  const auto foreground = program.pc();
  program.emit("nop");
  program.emit("nop");
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  fixture::run(machine, odd ? 5 : 4);
  machine.bus().write({.space = blofeld::DspSpace::x, .address = fixture::iprp, .value = 0x100});
  timer(machine);
  machine.cpu().step();
  require(machine.cpu().state().pc == foreground && machine.bus().interrupt_request(0),
          "Actual unmasking instruction clocks the timer into a pending source");
  constexpr unsigned setup_cycles = 10;
  require(machine.run_clock().native_cycle_units() == setup_cycles + static_cast<unsigned>(odd),
          "Five independent two-cycle immediate MOVEs and the optional one-cycle NOP");
  return foreground;
}

void check_delta(blofeld::DspMachine &machine,
                 const blofeld::DspStepPlan &plan,
                 const unsigned units,
                 const unsigned period,
                 const unsigned counter) {
  const auto before = plan.before.native_cycles;
  const auto edges = ((before & 1U) + units) / 2;
  require(machine.run_clock().native_cycle_units() == before + units &&
              machine.cpu().debug_state().native_cycles == before + units &&
              machine.run_clock().elapsed_ticks() ==
                  plan.start_ticks + static_cast<std::uint64_t>(units) * period &&
              machine.run_clock().timer_clk2_edges() == (before + units) / 2 &&
              machine.bus().timers().read(count0) == counter + edges,
          "Literal instruction costs advance actual native/clock counters and retained CLK2 phase");
}

struct VectorCase {
  const char *first;
  const char *second;
  unsigned units;
  unsigned instructions;
  bool long_entry;
  unsigned x0 = 0;
  unsigned y0 = 0;
  unsigned r0 = fixture::handler;
  std::optional<std::array<std::uint32_t, 2>> words = std::nullopt;
};

void check_native_case(const VectorCase &test, const bool odd, const bool pll) {
  blofeld::DspMachine machine(contract());
  const auto foreground = initialize(machine, odd, 0, test.r0);
  if (test.words) {
    fixture::ram(machine.bus(), fixture::timer_vector, *test.words);
  } else {
    vector(machine, fixture::timer_vector, test.first, test.second);
  }
  fixture::ram(machine.bus(), fixture::handler, fixture::assemble("rti"));
  if (pll) { machine.bus().pll().write(stage_pctl); }
  const auto before = machine.cpu().debug_state();
  const auto counter = machine.bus().timers().read(count0);
  const auto plan = machine.plan_step();
  const auto period = pll ? pll_period : initial_period;
  require(plan == machine.plan_step() && machine.cpu().debug_state() == before &&
              machine.bus().timers().read(count0) == counter &&
              machine.bus().interrupt_request(0) && plan.native_cycle_units == test.units &&
              plan.duration_ticks == static_cast<std::uint64_t>(test.units) * period &&
              plan.interrupt && plan.interrupt->address == fixture::timer_vector &&
              plan.interrupt->request.level == 0,
          "Two pure previews retain pending IRQ, CPU, timer and the literal vector cost");
  unsigned fetches = 0;
  machine.cpu().step({.observe = [&](const blofeld::DspNativeAccess &access) {
    if (access.kind != blofeld::DspNativeAccessKind::vector) { return; }
    require(access.target.address == fixture::timer_vector + fetches / 2 &&
                machine.bus().interrupt_request(0) &&
                machine.run_clock().elapsed_ticks() == plan.start_ticks,
            "Both actual vector reads precede acknowledgement and clock commit");
    ++fetches;
  }});
  const auto after = machine.cpu().debug_state();
  require(fetches == 4 && !machine.bus().interrupt_request(0) &&
              (machine.bus().timers().read(fixture::csr0, compare_flag) & compare_flag) == 0 &&
              after.native_instructions == before.native_instructions + test.instructions,
          "Exactly two real before/after reads acknowledge once; only executed words count");
  check_delta(machine, plan, test.units, period, counter);
  require((after.cpu.x & word_mask) == test.x0 && (after.cpu.y & word_mask) == test.y0 &&
              after.cpu.r.at(0) == fixture::handler,
          "Actual vector result for " + std::string(test.first) + "; " + test.second +
              ": x=" + number(after.cpu.x) + " y=" + number(after.cpu.y) +
              " r0=" + number(after.cpu.r.at(0)));
  if (test.long_entry) {
    require(after.cpu.pc == fixture::handler && after.cpu.sp == 1 &&
                after.cpu.stack_pc == foreground && after.cpu.stack_sr == before.cpu.sr &&
                (after.cpu.sr & fixture::mask_bits) == 0x100,
            "Native JSR retains the foreground return and original SR in a genuine long IRQ frame");
    const auto return_plan = machine.plan_step();
    const auto return_counter = machine.bus().timers().read(count0);
    require(return_plan.native_cycle_units == 3 && !return_plan.interrupt,
            "FM nominal RTI is three units and no pending IRQ remains");
    machine.cpu().step();
    check_delta(machine, return_plan, 3, period, return_counter);
    require(machine.cpu().state().pc == foreground && machine.cpu().state().sp == 0 &&
                machine.cpu().state().sr == before.cpu.sr,
            "Native RTI restores the interrupted foreground and its original SR");
  } else {
    require(after.cpu.pc == foreground && after.cpu.sp == 0,
            "Fast entry executes its vector and resumes without a synthetic stack frame");
  }
}

void test_vector_costs_and_phase() {
  // FM A-1/A-6/A-7: NOP/short MOVE=1, long immediate MOVE=2,
  // short/indirect JSR=3, absolute/pre-update JSR=4; ordinary RTI=3.
  constexpr std::array cases{
      VectorCase{
          .first = "nop", .second = "nop", .units = 2, .instructions = 2, .long_entry = false},
      VectorCase{.first = "move #1,x0",
                 .second = "move #2,y0",
                 .units = 2,
                 .instructions = 2,
                 .long_entry = false,
                 .x0 = fractional_one,
                 .y0 = fractional_two},
      VectorCase{.first = "move #>$123456,x0",
                 .second = "nop",
                 .units = 2,
                 .instructions = 1,
                 .long_entry = false,
                 .x0 = fixture::cookie},
      VectorCase{.first = "jsr $300",
                 .second = "illegal",
                 .units = 3,
                 .instructions = 1,
                 .long_entry = true},
      VectorCase{
          .first = "nop", .second = "jsr $300", .units = 4, .instructions = 2, .long_entry = true},
      VectorCase{.first = "JSR ea (absolute)",
                 .second = "extension",
                 .units = 4,
                 .instructions = 1,
                 .long_entry = true,
                 .words = std::array<std::uint32_t, 2>{absolute_jsr, fixture::handler}},
      VectorCase{.first = "jsr (r0)",
                 .second = "illegal",
                 .units = 3,
                 .instructions = 1,
                 .long_entry = true},
      VectorCase{.first = "jsr -(r0)",
                 .second = "illegal",
                 .units = 4,
                 .instructions = 1,
                 .long_entry = true,
                 .x0 = 0,
                 .y0 = 0,
                 .r0 = fixture::handler + 1}};
  for (const bool odd : {false, true}) {
    for (const bool pll : {false, true}) {
      for (const auto &test : cases) { check_native_case(test, odd, pll); }
    }
  }
}

void test_unknown_and_unsupported_vectors() {
  blofeld::DspMachine machine(contract());
  static_cast<void>(initialize(machine));
  const auto unknown = machine.checkpoint();
  const auto before = machine.cpu().debug_state();
  const auto ticks = machine.run_clock().elapsed_ticks();
  const auto error = fixture::rejects([&] { static_cast<void>(machine.plan_step()); },
                                      "Unknown vector words must reject pure preview");
  require(error.contains("0x000054") && !machine.timing_faulted() &&
              machine.cpu().debug_state() == before && machine.bus().interrupt_request(0),
          "Unknown vector preview preserves the healthy pending source and reports its address");
  fixture::ram(machine.bus(), fixture::timer_vector, fixture::assemble("jsr $300"));
  static_cast<void>(fixture::rejects([&] { machine.cpu().step(); },
                                     "Even unused second vector words must be known"));
  require(machine.timing_faulted() && machine.cpu().debug_state() == before &&
              machine.run_clock().elapsed_ticks() == ticks && machine.bus().interrupt_request(0),
          "Unknown second fetch rejects before native entry, acknowledgement or elapsed time");
  constexpr std::array rejected{"rti",
                                "rts",
                                "rep #2",
                                "jmp $300",
                                "illegal",
                                "reset",
                                "andi #$ff,mr",
                                "ori #1,ccr",
                                "move #1,la",
                                "move #1,lc",
                                "move #1,sp",
                                "move #1,sc",
                                "move #1,ssh",
                                "move #1,ssl",
                                "move ssh,x0",
                                "btst #0,ssh",
                                "bset #0,ssl"};
  for (const auto *instruction : rejected) {
    machine.restore(unknown);
    vector(machine, fixture::timer_vector, instruction);
    static_cast<void>(fixture::rejects([&] { machine.cpu().step(); },
                                       "Unsupported vector instructions must reject before entry"));
    require(machine.cpu().debug_state() == before && machine.bus().interrupt_request(0) &&
                machine.run_clock().elapsed_ticks() == ticks,
            "Unsupported vector retains the independently known interrupted prefix");
  }
  machine.restore(unknown);
  fixture::ram(machine.bus(), fixture::timer_vector, fixture::assemble("nop"));
  fixture::ram(machine.bus(), fixture::timer_vector + 1, fixture::assemble("move #>$123456,x0"));
  static_cast<void>(fixture::rejects([&] { machine.cpu().step(); },
                                     "A double-word instruction cannot start at vector word two"));
  require(machine.cpu().debug_state() == before && machine.bus().interrupt_request(0),
          "Length failure precedes execution of the first vector NOP");
}

void test_scheduled_mutation_and_restore() {
  for (const unsigned changed_word : {0U, 1U}) {
    blofeld::DspMachine machine(contract());
    static_cast<void>(initialize(machine, true, relocated_base));
    const auto address = relocated_base + fixture::timer_vector;
    vector(machine, address, "nop");
    const auto saved = machine.checkpoint();
    const auto plan = machine.plan_step();
    blofeld::DspScheduledRun schedule(
        {.observation = blofeld::DspStepObservation::reference_completion_boundary}, machine);
    const auto end = plan.start_ticks + plan.duration_ticks;
    require(schedule.advance_to(machine, end - 1, blofeld::DspScheduleBoundary::through, 1) == 0,
            "Scheduler retains a pinned vector before its declared completion");
    const auto pending = schedule.checkpoint(machine);
    fixture::ram(machine.bus(), address + changed_word, fixture::assemble("move #1,x0"));
    require(machine.plan_step().native_cycle_units == plan.native_cycle_units &&
                machine.plan_step() != plan,
            "Same-cost word mutation changes the scheduled vector identity");
    const auto error = fixture::rejects(
        [&] { schedule.advance_to(machine, end, blofeld::DspScheduleBoundary::through, 1); },
        "Changing either pinned word must reject before native dispatch");
    require(error.contains("changed before") && schedule.faulted() &&
                machine.cpu().debug_state() == plan.before && machine.bus().interrupt_request(0),
            "Schedule rejection retains both original pending source and native prefix");
    machine.restore(saved);
    schedule.restore(pending, machine);
    require(schedule.advance_to(machine, end, blofeld::DspScheduleBoundary::through, 1) == 1,
            "Coordinated snapshot restore recovers the original vector deadline");
    const auto completed = machine.cpu().debug_state();
    const auto completed_clock = machine.run_clock().elapsed_ticks();
    machine.restore(saved);
    schedule.restore(pending, machine);
    require(schedule.advance_to(machine, end, blofeld::DspScheduleBoundary::through, 1) == 1 &&
                machine.cpu().debug_state() == completed &&
                machine.run_clock().elapsed_ticks() == completed_clock,
            "Pending IRQ checkpoint reproduces native state, time and CLK2 residue");
  }
}

void test_nested_native_interrupts() {
  blofeld::DspMachine machine(contract());
  const auto foreground = initialize(machine);
  vector(machine, fixture::timer_vector, "jsr $300");
  vector(machine, fixture::second_vector, "jsr $320");
  fixture::ram(machine.bus(), fixture::handler, fixture::assemble("nop"));
  fixture::ram(machine.bus(), fixture::handler + 1, fixture::assemble("rti"));
  fixture::ram(machine.bus(), fixture::nested_handler, fixture::assemble("rti"));
  machine.cpu().step();
  require(machine.cpu().state().sp == 1 && machine.cpu().state().pc == fixture::handler,
          "Actual first JSR enters the low-priority handler");
  machine.bus().write(
      {.space = blofeld::DspSpace::x, .address = fixture::iprp, .value = higher_priorities});
  timer(machine, fixture::csr2);
  machine.cpu().step();
  const auto nested_plan = machine.plan_step();
  require(machine.bus().interrupt_request(1) && nested_plan.interrupt &&
              nested_plan.interrupt->address == fixture::second_vector,
          "Handler NOP clocks a genuine higher-priority timer request");
  const auto frozen = machine.checkpoint();
  const auto start = machine.run_clock().native_cycle_units();
  fixture::run(machine, 3);
  const auto completed = machine.cpu().debug_state();
  constexpr unsigned nested_and_returns = 9;
  require(completed.cpu.pc == foreground && completed.cpu.sp == 0 &&
              completed.cpu.sr == fixture::ordinary_status &&
              machine.run_clock().native_cycle_units() == start + nested_and_returns,
          "Nested JSR and two genuine RTIs consume 3+3+3 units and unwind both native frames");
  const auto ticks = machine.run_clock().elapsed_ticks();
  const auto count = machine.bus().timers().read(count0);
  machine.restore(frozen);
  fixture::run(machine, 3);
  require(machine.cpu().debug_state() == completed &&
              machine.run_clock().elapsed_ticks() == ticks &&
              machine.bus().timers().read(count0) == count,
          "Snapshot with a pending nested IRQ retains full stack, source and shared clock phase");
}

void test_repeat_defers_pending_interrupt() {
  blofeld::DspMachine machine(contract());
  DspProgram program;
  program.immediate("sr", fixture::ordinary_status);
  program.emit("rep #2");
  program.emit("inc a");
  program.emit("rep #1");
  program.emit("inc b");
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = blofeld::DspSpace::x, .address = fixture::iprp, .value = 0x100});
  vector(machine, fixture::timer_vector, "nop");
  fixture::run(machine, 2);
  timer(machine);
  machine.cpu().step();
  const auto repeated = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(repeated);
    require(!machine.plan_step().interrupt, "Pending timer cannot interrupt the active REP body");
    machine.cpu().step();
    require(
        machine.bus().interrupt_request(0) && !machine.plan_step().interrupt,
        "Adjacent REP remains deferred after completion while the original source stays pending");
    machine.cpu().step();
    require(!machine.plan_step().interrupt, "The adjacent one-iteration REP also remains atomic");
    machine.cpu().step();
    const auto plan = machine.plan_step();
    require(
        plan.interrupt && plan.native_cycle_units == 2 && machine.cpu().state().pc == foreground &&
            machine.cpu().state().a == 2 && machine.cpu().state().b == 1,
        "Actual REP arithmetic completes before the still-pending fast vector becomes admissible");
    machine.cpu().step();
    require(machine.cpu().state().pc == foreground && !machine.bus().interrupt_request(0),
            "The eventual vector acknowledges only after both adjacent native repeats finish");
  }
}

void test_irq_pll_command_latching() {
  blofeld::DspMachine machine(contract());
  static_cast<void>(initialize(machine));
  vector(machine, fixture::timer_vector, "movep #>$02e112,x:<<$fffffd");
  const auto plan = machine.plan_step();
  const auto counter = machine.bus().timers().read(count0);
  require(plan.native_cycle_units == 2 && plan.duration_ticks == std::uint64_t{2} * initial_period,
          "The two-word vector command uses the prior PLL rate for both nominal units");
  machine.cpu().step();
  check_delta(machine, plan, 2, initial_period, counter);
  const auto next = machine.plan_step();
  require(!next.interrupt && next.native_cycle_units == 1 && next.duration_ticks == pll_period,
          "The actual vector PCTL command changes the following foreground instruction rate");
  machine.cpu().step();
  require(machine.run_clock().elapsed_ticks() == next.start_ticks + pll_period,
          "The next native foreground NOP commits at the new declared PLL ratio");
}

void test_contract_separation() {
  blofeld::DspMachine machine(contract());
  static_cast<void>(initialize(machine));
  const auto saved = machine.checkpoint();
  auto legacy = contract();
  legacy.interrupts = blofeld::DspInterruptTiming::unqualified;
  blofeld::DspMachine old_owner(legacy);
  static_cast<void>(
      fixture::rejects([&] { old_owner.restore(saved); },
                       "A vector-estimate snapshot requires the same timing contract"));
}
}

int main() {
  try {
    test_vector_costs_and_phase();
    test_unknown_and_unsupported_vectors();
    test_scheduled_mutation_and_restore();
    test_nested_native_interrupts();
    test_repeat_defers_pending_interrupt();
    test_irq_pll_command_latching();
    test_contract_separation();
    std::cout
        << "Nominal vector costs, native entry/RTI, IRQ snapshots and strict failures passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
