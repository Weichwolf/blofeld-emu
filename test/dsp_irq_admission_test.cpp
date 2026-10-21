// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using blofeld::test::require;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned period = 45;
constexpr unsigned csr = 0xffff8f;
constexpr unsigned capture = 0x165;
constexpr unsigned vector_address = 0x54;
constexpr unsigned mid_instruction = 22;
constexpr unsigned foreground_value = 5;
constexpr unsigned irq_value = 7;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned payload = 0x123456;
constexpr unsigned shi_vector = 0x44;
constexpr unsigned priorities = 0x10c;
constexpr unsigned raised_timer_priority = 0x300;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned shi_config = 0x1009;
constexpr unsigned word_bits = 24;

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates,
          .interrupts = blofeld::DspInterruptTiming::native_vector_estimates};
}

blofeld::DspScheduleInputs schedule_inputs() {
  return {.observation = blofeld::DspStepObservation::reference_completion_boundary,
          .irq_admission = blofeld::DspIrqAdmission::reference_prepare_boundary};
}

void install_vector(blofeld::DspMachine &machine, const unsigned address, const char *first) {
  const auto words = fixture::assemble(first);
  require(words.size() == 1, "Own prepared-admission vector starts with one word");
  fixture::ram(machine.bus(), address, words);
  fixture::ram(machine.bus(), address + 1, fixture::assemble("nop"));
}

void drive(blofeld::DspMachine &machine, const bool level) {
  machine.bus().timers().set_tio_drive(0, level);
  machine.bus().timers().observe_synchronized_tio(0, level);
}

unsigned initialize(blofeld::DspMachine &machine, const bool odd = false) {
  blofeld::test::DspProgram program;
  program.immediate("r1", 0);
  program.immediate("r2", 0);
  program.immediate("sr", ordinary_status);
  if (odd) { program.emit("nop"); }
  const auto foreground = program.pc();
  program.emit("move #<5,r2");
  program.emit("nop");
  program.emit("nop");
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  // The CPU, not a caller-supplied cycle chunk, qualifies SHI's enable delay.
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = blofeld::DspSpace::x, .address = hcsr, .value = shi_config});
  fixture::run(machine, odd ? 4 : 3);
  machine.bus().write(
      {.space = blofeld::DspSpace::x, .address = fixture::iprp, .value = priorities});
  drive(machine, true);
  machine.bus().timers().write({.address = csr - 1, .value = 0});
  machine.bus().timers().write({.address = csr - 2, .value = 1});
  machine.bus().timers().write({.address = csr, .value = capture});
  install_vector(machine, vector_address, "move #<7,r1");
  return foreground;
}

struct Edge {
  blofeld::DspMachine &machine;
  std::uint64_t ticks;
  bool delivered = false;

  blofeld::DspExternalEvents source(const blofeld::DspExternalOrder order) {
    return {.next_ticks = [this]() -> std::optional<std::uint64_t> {
              return delivered ? std::nullopt : std::optional(ticks);
            },
            .observe =
                [this](const std::uint64_t observed) {
                  require(observed == ticks && !delivered, "Exactly one declared TIO event");
                  drive(machine, false);
                  delivered = true;
                },
            .order = order,
            .maximum_events = 1};
  }
};

void check_foreground_then_irq(const unsigned offset,
                               const bool odd,
                               const blofeld::DspExternalOrder order) {
  blofeld::DspMachine machine(contract());
  const auto foreground = initialize(machine, odd);
  const auto before = machine.cpu().debug_state();
  const auto start = machine.run_clock().elapsed_ticks();
  blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
  Edge edge{.machine = machine, .ticks = start + offset};
  const auto external = edge.source(order);
  require(
      schedule.advance_to(
          machine, start + period, blofeld::DspScheduleBoundary::through, 1, {}, {}, external) == 1,
      "The admitted foreground instruction completes despite a later real capture IRQ");
  const auto after = machine.cpu().debug_state();
  require(edge.delivered && after.cpu.pc == foreground + 1 &&
              after.cpu.r.at(2) == foreground_value && after.cpu.r.at(1) == 0 &&
              after.cpu.sp == 0 && after.native_instructions == before.native_instructions + 1 &&
              after.native_cycles == before.native_cycles + 1 &&
              machine.run_clock().elapsed_ticks() == start + period &&
              machine.bus().interrupt_request(0) && schedule.completed_external_events() == 1,
          "Literal foreground effects/time precede IRQ entry; the real source remains pending");
  require(schedule.advance_to(machine,
                              start + (std::uint64_t{3} * period),
                              blofeld::DspScheduleBoundary::through,
                              1,
                              {},
                              {},
                              external) == 1,
          "The next admitted step executes both actual fast-vector instructions");
  const auto completed = machine.cpu().debug_state();
  require(
      completed.cpu.r.at(1) == irq_value && completed.cpu.r.at(2) == foreground_value &&
          completed.cpu.pc == foreground + 1 && completed.cpu.sp == 0 &&
          completed.native_instructions == before.native_instructions + 3 &&
          completed.native_cycles == before.native_cycles + 3 &&
          machine.run_clock().native_cycle_units() == before.native_cycles + 3 &&
          machine.run_clock().timer_clk2_edges() == (before.native_cycles + 3) / 2 &&
          !machine.bus().interrupt_request(0),
      "Fast-vector data, one acknowledgement and CLK2 residue follow independent 1+2-unit costs");
}

void test_edges_and_ties() {
  constexpr std::array offsets{1U, mid_instruction, period - 1};
  for (const bool odd : {false, true}) {
    for (const auto offset : offsets) {
      check_foreground_then_irq(offset, odd, blofeld::DspExternalOrder::reject_tie);
    }
    for (const auto order : {blofeld::DspExternalOrder::dsp_before_external,
                             blofeld::DspExternalOrder::external_before_dsp}) {
      check_foreground_then_irq(period, odd, order);
    }
  }
}

void test_inputs_at_prepare_boundary() {
  for (const bool odd : {false, true}) {
    blofeld::DspMachine machine(contract());
    const auto foreground = initialize(machine, odd);
    const auto before = machine.cpu().debug_state();
    const auto start = machine.run_clock().elapsed_ticks();
    blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
    Edge edge{.machine = machine, .ticks = start};
    const auto external = edge.source(blofeld::DspExternalOrder::reject_tie);
    require(schedule.advance_to(machine,
                                start + (std::uint64_t{2} * period),
                                blofeld::DspScheduleBoundary::through,
                                1,
                                {},
                                {},
                                external) == 1 &&
                edge.delivered && machine.cpu().state().pc == foreground &&
                machine.cpu().state().r.at(1) == irq_value && machine.cpu().state().r.at(2) == 0 &&
                machine.cpu().debug_state().native_cycles == before.native_cycles + 2 &&
                machine.run_clock().elapsed_ticks() == start + (std::uint64_t{2} * period),
            "An input already due at the current boundary precedes admission, without a phantom "
            "foreground step");
  }
}

void test_completion_edges_without_chunk_boundary() {
  constexpr std::array offsets{period, period + 1};
  for (const auto offset : offsets) {
    for (const auto order : {blofeld::DspExternalOrder::dsp_before_external,
                             blofeld::DspExternalOrder::external_before_dsp}) {
      blofeld::DspMachine machine(contract());
      const auto foreground = initialize(machine);
      const auto before = machine.cpu().debug_state();
      const auto start = machine.run_clock().elapsed_ticks();
      blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
      Edge edge{.machine = machine, .ticks = start + offset};
      const auto external = edge.source(order);
      const bool late = offset > period;
      const unsigned steps = late ? 3U : 2U;
      const unsigned units = late ? 4U : 3U;
      const auto end = start + (static_cast<std::uint64_t>(units) * period);
      require(schedule.advance_to(
                  machine, end, blofeld::DspScheduleBoundary::through, steps, {}, {}, external) ==
                      steps &&
                  edge.delivered && machine.cpu().state().pc == foreground + (late ? 2U : 1U) &&
                  machine.cpu().state().r.at(1) == irq_value &&
                  machine.cpu().state().r.at(2) == foreground_value &&
                  machine.cpu().debug_state().native_cycles == before.native_cycles + units &&
                  machine.run_clock().elapsed_ticks() == end,
              "Completion ties precede the next admission; later edges retain the already begun "
              "next instruction");
    }
  }
}

void test_legacy_and_rejected_ties() {
  for (const bool legacy : {false, true}) {
    blofeld::DspMachine machine(contract());
    static_cast<void>(initialize(machine));
    auto inputs = schedule_inputs();
    if (legacy) { inputs.irq_admission = blofeld::DspIrqAdmission::reject_changes; }
    blofeld::DspScheduledRun schedule(inputs, machine);
    const auto before = machine.cpu().debug_state();
    const auto start = machine.run_clock().elapsed_ticks();
    Edge edge{.machine = machine, .ticks = start + (legacy ? mid_instruction : period)};
    const auto external = edge.source(blofeld::DspExternalOrder::reject_tie);
    const auto error = fixture::rejects(
        [&] {
          schedule.advance_to(
              machine, start + period, blofeld::DspScheduleBoundary::through, 1, {}, {}, external);
        },
        "Legacy mid-step changes and unqualified simultaneous events remain rejected");
    require(schedule.faulted() && !machine.timing_faulted() &&
                machine.cpu().debug_state() == before &&
                machine.run_clock().elapsed_ticks() == start && edge.delivered == legacy &&
                (legacy ? error.contains("changed before") : error.contains("simultaneous")),
            "Rejection retains exact native/time prefix and distinguishes delivered inputs");
  }
}

void test_pending_foreground_snapshot() {
  blofeld::DspMachine machine(contract());
  const auto foreground = initialize(machine, true);
  blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
  const auto start = machine.run_clock().elapsed_ticks();
  Edge edge{.machine = machine, .ticks = start + mid_instruction};
  const auto external = edge.source(blofeld::DspExternalOrder::reject_tie);
  require(schedule.advance_to(
              machine, edge.ticks, blofeld::DspScheduleBoundary::through, 1, {}, {}, external) ==
                  0 &&
              edge.delivered && machine.bus().interrupt_request(0),
          "A delivered IRQ can coexist with the still-admitted foreground plan");
  const auto owner = machine.checkpoint();
  const auto pending = schedule.checkpoint(machine);
  std::optional<blofeld::DspDebugState> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(owner);
    schedule.restore(pending, machine);
    require(schedule.advance_to(machine,
                                start + (std::uint64_t{3} * period),
                                blofeld::DspScheduleBoundary::through,
                                2,
                                {},
                                {},
                                external) == 2,
            "Snapshot retains the foreinstruction admission and later native fast vector");
    const auto state = machine.cpu().debug_state();
    require(state.cpu.pc == foreground + 1 && state.cpu.r.at(1) == irq_value &&
                state.cpu.r.at(2) == foreground_value,
            "Replayed effects come from the actual foreground and vector");
    if (expected) { require(state == *expected, "Whole native snapshot replay differs"); }
    expected = state;
  }
}

void test_snapshot_before_edge() {
  blofeld::DspMachine machine(contract());
  static_cast<void>(initialize(machine));
  blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
  const auto start = machine.run_clock().elapsed_ticks();
  Edge edge{.machine = machine, .ticks = start + mid_instruction};
  const auto external = edge.source(blofeld::DspExternalOrder::reject_tie);
  require(
      schedule.advance_to(
          machine, edge.ticks - 1, blofeld::DspScheduleBoundary::through, 1, {}, {}, external) ==
              0 &&
          !edge.delivered,
      "Snapshot is before a pinned edge and before foreground completion");
  const auto owner = machine.checkpoint();
  const auto pending = schedule.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(owner);
    edge.delivered = false;
    schedule.restore(pending, machine);
    require(schedule.advance_to(machine,
                                start + (std::uint64_t{3} * period),
                                blofeld::DspScheduleBoundary::through,
                                2,
                                {},
                                {},
                                external) == 2 &&
                edge.delivered && schedule.completed_external_events() == 1 &&
                machine.cpu().state().r.at(1) == irq_value &&
                machine.cpu().state().r.at(2) == foreground_value &&
                machine.run_clock().elapsed_ticks() == start + (std::uint64_t{3} * period),
            "Owner, schedule and caller cursor jointly replay edge, foreground and actual IRQ");
  }
}

void test_native_mask_changes_before_next_admission() {
  blofeld::DspMachine machine(contract());
  const auto foreground = initialize(machine);
  constexpr unsigned masked = 0xc00300;
  const auto mask_words = fixture::assemble("move #>$c00300,sr");
  machine.bus().install_rom({.start = foreground, .words = mask_words});
  const auto start = machine.run_clock().elapsed_ticks();
  blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
  Edge edge{.machine = machine, .ticks = start + mid_instruction};
  const auto external = edge.source(blofeld::DspExternalOrder::reject_tie);
  require(
      schedule.advance_to(machine,
                          start + (std::uint64_t{2} * period),
                          blofeld::DspScheduleBoundary::through,
                          1,
                          {},
                          {},
                          external) == 1 &&
          machine.cpu().state().sr == masked && machine.bus().interrupt_request(0) &&
          !machine.plan_step().interrupt,
      "The actual admitted mask instruction completes before later IRQ eligibility is evaluated");
  const auto unmask_words = fixture::assemble("move #>$c00000,sr");
  machine.bus().install_rom({.start = foreground + 2, .words = unmask_words});
  constexpr std::array<std::uint32_t, 1> suffix{0};
  machine.bus().install_rom({.start = foreground + 4, .words = suffix});
  require(
      schedule.advance_to(machine,
                          start + (std::uint64_t{4} * period),
                          blofeld::DspScheduleBoundary::through,
                          1,
                          {},
                          {},
                          external) == 1 &&
          machine.cpu().state().sr == ordinary_status && machine.plan_step().interrupt,
      "A genuine unmask instruction makes the still-pending source eligible at the next boundary");
  require(schedule.advance_to(machine,
                              start + (std::uint64_t{6} * period),
                              blofeld::DspScheduleBoundary::through,
                              1,
                              {},
                              {},
                              external) == 1 &&
              machine.cpu().state().r.at(1) == irq_value && !machine.bus().interrupt_request(0),
          "The deferred source is serviced by its actual fast vector after native unmasking");
}

void send(blofeld::DspShi &shi) {
  shi.observe_spi({.mosi = false});
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool level = ((payload >> (word_bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

void test_admitted_irq_retains_priority_order() {
  blofeld::DspMachine machine(contract());
  const auto foreground = initialize(machine);
  install_vector(machine, vector_address, "jsr $300");
  install_vector(machine, shi_vector, "jsr $320");
  fixture::ram(machine.bus(), fixture::handler, fixture::assemble("rti"));
  fixture::ram(machine.bus(), fixture::nested_handler, fixture::assemble("movep x:<<$ffff94,x0"));
  fixture::ram(machine.bus(), fixture::nested_handler + 1, fixture::assemble("rti"));
  drive(machine, false);
  const auto start = machine.run_clock().elapsed_ticks();
  const auto before_units = machine.run_clock().native_cycle_units();
  blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
  bool arrived = false;
  const auto event = start + mid_instruction;
  const blofeld::DspExternalEvents external{
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        return arrived ? std::nullopt : std::optional(event);
      },
      .observe =
          [&](const std::uint64_t ticks) {
            require(ticks == event && !arrived, "One explicit SHI word arrival during timer entry");
            send(machine.bus().shi());
            arrived = true;
          },
      .order = blofeld::DspExternalOrder::reject_tie,
      .maximum_events = 1};
  const auto first_steps = schedule.advance_to(machine,
                                               start + (std::uint64_t{3} * period),
                                               blofeld::DspScheduleBoundary::through,
                                               1,
                                               {},
                                               {},
                                               external);
  const auto higher = machine.bus().interrupt_request(1);
  require(first_steps == 1 && machine.cpu().state().pc == fixture::handler &&
              machine.cpu().state().sp == 1 && higher && higher->vector == shi_vector,
          "A later higher-priority source cannot replace the already admitted timer JSR");
  const auto low_owner = machine.checkpoint();
  const auto low_schedule = schedule.checkpoint(machine);
  require(schedule.advance_to(machine,
                              start + (std::uint64_t{6} * period),
                              blofeld::DspScheduleBoundary::through,
                              1,
                              {},
                              {},
                              external) == 1 &&
              machine.cpu().state().pc == fixture::nested_handler && machine.cpu().state().sp == 2,
          "The preserved higher-priority source enters a genuine nested JSR on the next step");
  constexpr unsigned remaining_units = 13;
  require(schedule.advance_to(machine,
                              start + (static_cast<std::uint64_t>(remaining_units) * period),
                              blofeld::DspScheduleBoundary::through,
                              3,
                              {},
                              {},
                              external) == 3,
          "Actual HRX read and two RTIs consume one plus three plus three units");
  const auto final = machine.cpu().debug_state();
  require(final.cpu.pc == foreground && final.cpu.sp == 0 && final.cpu.sr == ordinary_status &&
              (final.cpu.x & blofeld::test::word_mask) == payload &&
              final.native_cycles == before_units + remaining_units &&
              !machine.bus().interrupt_request(0),
          "Two native frames unwind, original HRX data is read once and both real sources clear");
  machine.restore(low_owner);
  schedule.restore(low_schedule, machine);
  require(schedule.advance_to(machine,
                              start + (static_cast<std::uint64_t>(remaining_units) * period),
                              blofeld::DspScheduleBoundary::through,
                              4,
                              {},
                              {},
                              external) == 4 &&
              machine.cpu().debug_state() == final,
          "A checkpoint after the lower entry preserves the later source and native unwind");
}

void test_admitted_source_and_word_changes_reject() {
  for (unsigned scenario = 0; scenario < 4; ++scenario) {
    blofeld::DspMachine machine(contract());
    static_cast<void>(initialize(machine));
    install_vector(machine, vector_address, "jsr $300");
    drive(machine, false);
    const auto before = machine.cpu().debug_state();
    const auto plan = machine.plan_step();
    const auto start = machine.run_clock().elapsed_ticks();
    blofeld::DspScheduledRun schedule(schedule_inputs(), machine);
    const auto error = fixture::rejects(
        [&] {
          schedule.advance_to(
              machine,
              start + (std::uint64_t{3} * period),
              blofeld::DspScheduleBoundary::through,
              1,
              [&](const blofeld::DspScheduledStep &) {
                if (scenario < 2) {
                  fixture::ram(machine.bus(),
                               vector_address + scenario,
                               fixture::assemble(scenario == 0 ? "jsr $320" : "move #<1,r1"));
                } else if (scenario == 2) {
                  machine.bus().timers().write({.address = csr, .value = 0});
                } else {
                  machine.bus().write({.space = blofeld::DspSpace::x,
                                       .address = fixture::iprp,
                                       .value = raised_timer_priority});
                }
              });
        },
        "Prepared vector/source/priority mutation must reject before dispatch");
    constexpr std::array reasons{"admitted instruction changed",
                                 "admitted instruction changed",
                                 "source withdrew",
                                 "priority changed"};
    const bool intended_error = error.contains(reasons.at(scenario));
    require(intended_error && schedule.faulted() && !machine.timing_faulted() &&
                machine.cpu().debug_state() == before &&
                machine.run_clock().elapsed_ticks() == start && plan.interrupt.has_value(),
            "Mutations cannot silently redirect an admitted IRQ or commit its nominal time");
    if (scenario < 2) {
      const auto changed = machine.plan_step();
      require(changed.native_cycle_units == plan.native_cycle_units &&
                  changed.interrupt != plan.interrupt && machine.bus().interrupt_request(0),
              "Both word replacements retain three units but change strict vector identity");
    }
  }
}

void test_contract_mismatch() {
  auto run = contract();
  run.interrupts = blofeld::DspInterruptTiming::unqualified;
  blofeld::DspMachine old(run);
  static_cast<void>(
      fixture::rejects([&] { const blofeld::DspScheduledRun schedule(schedule_inputs(), old); },
                       "Prepared admission requires an explicit vector-cost owner"));
  blofeld::DspMachine machine(contract());
  static_cast<void>(initialize(machine));
  const blofeld::DspScheduledRun admitted(schedule_inputs(), machine);
  const auto saved = admitted.checkpoint(machine);
  blofeld::DspScheduledRun legacy(
      {.observation = blofeld::DspStepObservation::reference_completion_boundary}, machine);
  static_cast<void>(fixture::rejects([&] { legacy.restore(saved, machine); },
                                     "Schedule snapshots retain the admission policy"));
}
}

int main() {
  try {
    test_edges_and_ties();
    test_inputs_at_prepare_boundary();
    test_completion_edges_without_chunk_boundary();
    test_legacy_and_rejected_ties();
    test_pending_foreground_snapshot();
    test_snapshot_before_edge();
    test_native_mask_changes_before_next_admission();
    test_admitted_irq_retains_priority_order();
    test_admitted_source_and_word_changes_reject();
    test_contract_mismatch();
    std::cout << "Prepared IRQ admission, source priority, native effects and snapshots passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
