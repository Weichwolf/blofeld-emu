// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_fetch_schedule_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::fetch_events {
DspFetchScheduleInputs inputs() {
  return {.dsp = {.extal_hz = extal,
                  .timebase_hz = timebase,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory,
                  .resets = DspFetchResetTiming::reference_cycle_boundary}};
}

DspFetchCycleInputs cycle_inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspMachine &prepare(DspMachine &machine) {
  DspProgram code;
  for (unsigned i = 0; i < nops; ++i) { code.emit("nop"); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return machine;
}

DspFetchExternalEvents Source::events() {
  return {.identity = identity,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            return cursor < ticks.size() ? std::optional{ticks[cursor]} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                require(cursor < ticks.size() && ticks[cursor] == at,
                        "Actual external cursor/time matches");
                if (log) { log->push_back("E" + std::to_string(at)); }
                if (machine) { machine->bus().timers().set_tio_drive(0, (cursor & 1U) != 0); }
                ++cursor;
              },
          .order = order,
          .maximum_events = event_budget};
}

namespace {
void ordered(const DspExternalOrder order) {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  std::vector<std::string> log;
  Source source{
      .ticks = {0, 1, period, period + 1, 2 * period, (2 * period) + 2, 3 * period, horizon},
      .log = &log,
      .machine = &machine,
      .order = order};
  unsigned retired = 0;
  const auto after = [&](const DspFetchCycleResult &cycle) {
    log.push_back("D" + std::to_string(cycle.completion_ticks));
    require(cycle.period_ticks == period &&
                cycle.completion_ticks == cycle.completed_cycles * period,
            "Independent bypass period and literal cycle census retain every native edge");
    require(machine.cpu().state().a == 0 && machine.cpu().state().sr == initial_status,
            "Actual native NOPs preserve accumulator and reset flags");
    if (!cycle.pipeline) { throw Fault("Expected automatic native pipeline observation"); }
    if (cycle.pipeline->retirement_published) {
      require(cycle.pipeline->execute2 && cycle.pipeline->execute2->address == boot + retired,
              "Native NOP retirements preserve independent sequential original fetch addresses");
      ++retired;
    }
  };
  require(schedule.advance_to({.horizon = {.ticks = horizon}, .maximum_cycles = cycles},
                              cycle_inputs,
                              after,
                              source.events()) == cycles,
          "All 32 genuine periods execute alongside asynchronous pin observations");
  std::vector<std::string> expected{"E0", "E1"};
  if (order == DspExternalOrder::external_before_dsp) {
    expected.insert(expected.end(), {"E45", "D45", "E46", "E90", "D90", "E92", "E135", "D135"});
  } else {
    expected.insert(expected.end(), {"D45", "E45", "E46", "D90", "E90", "E92", "D135", "E135"});
  }
  require(log.size() == cycles + source.ticks.size() &&
              std::equal(expected.begin(), expected.end(), log.begin()) &&
              schedule.state().external_events == source.ticks.size() &&
              schedule.run().elapsed_ticks() == horizon &&
              schedule.run().timer_clk2_edges() == cycles / 2 && retired > 0,
          "Literal event prefixes, complete counts and timer edges agree independently");
  source.ticks.push_back(horizon + 1);
  require(schedule.advance_to({.horizon = {.ticks = horizon + period}, .maximum_cycles = 1},
                              cycle_inputs,
                              after,
                              source.events()) == 1 &&
              source.cursor == 9,
          "A bound exhausted source can append genuine future input without changing its identity");
}

void boundary_and_replay() {
  for (const auto order :
       {DspExternalOrder::external_before_dsp, DspExternalOrder::dsp_before_external}) {
    DspMachine machine;
    DspFetchScheduledRun schedule(inputs(), prepare(machine));
    Source source{.ticks = {period}, .machine = &machine, .order = order};
    const DspFetchHorizon cut{
        .ticks = period,
        .native = order == DspExternalOrder::external_before_dsp ? DspScheduleBoundary::before
                                                                 : DspScheduleBoundary::through,
        .external = order == DspExternalOrder::external_before_dsp ? DspScheduleBoundary::through
                                                                   : DspScheduleBoundary::before};
    static_cast<void>(schedule.advance_to(
        {.horizon = cut, .maximum_cycles = 1}, cycle_inputs, {}, source.events()));
    require(schedule.run().completed_cycles() ==
                    (order == DspExternalOrder::external_before_dsp ? 0 : 1) &&
                source.cursor == (order == DspExternalOrder::external_before_dsp ? 1 : 0),
            "A mixed boundary retains exactly the selected real event before the MCU phase");
    const auto checkpoint = schedule.checkpoint();
    const auto cursor = source.cursor;
    static_cast<void>(schedule.advance_to({.horizon = {.ticks = horizon}, .maximum_cycles = cycles},
                                          cycle_inputs,
                                          {},
                                          source.events()));
    const auto cpu = machine.cpu().debug_state();
    const auto state = schedule.state();
    for (unsigned replay = 0; replay < 2; ++replay) {
      schedule.restore(checkpoint);
      source.cursor = cursor;
      static_cast<void>(
          schedule.advance_to({.horizon = {.ticks = horizon}, .maximum_cycles = cycles},
                              cycle_inputs,
                              {},
                              source.events()));
      require(machine.cpu().debug_state() == cpu && schedule.state() == state,
              "Trusted native/pipeline snapshots replay both partial boundary orders with restored "
              "cursor");
    }
  }
}

void reset() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  schedule.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  constexpr unsigned held_end = 91;
  constexpr unsigned recovery_end = 226;
  constexpr unsigned first_cycle = 271;
  constexpr unsigned early_pin = 20;
  constexpr unsigned recovery_pin = 140;
  constexpr unsigned later_pin = 181;
  Source source{.ticks = {early_pin, held_end, recovery_pin, later_pin, recovery_end},
                .machine = &machine};
  static_cast<void>(schedule.advance_to({.horizon = {.ticks = held_end}}, {}, {}, source.events()));
  require(schedule.run().elapsed_ticks() == held_end && schedule.run().completed_cycles() == 0,
          "Held reset retains exact EXTAL fractions and pin events without native periods");
  const auto held = schedule.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    schedule.restore(held);
    source.cursor = 2;
    schedule.release_hardware_reset();
    static_cast<void>(schedule.advance_to(
        {.horizon = {.ticks = recovery_end, .native = DspScheduleBoundary::before}},
        {},
        {},
        source.events()));
    require(schedule.run().elapsed_ticks() == recovery_end - 1 &&
                schedule.run().hardware_reset_active() && source.cursor == 5,
            "An event at recovery completion can precede the still excluded real release edge");
    static_cast<void>(
        schedule.advance_to({.horizon = {.ticks = recovery_end}}, {}, {}, source.events()));
    require(schedule.run().elapsed_ticks() == recovery_end &&
                !schedule.run().hardware_reset_active(),
            "The remaining single recovery tick completes without a native instruction");
    static_cast<void>(schedule.advance_to({.horizon = {.ticks = first_cycle}, .maximum_cycles = 1},
                                          cycle_inputs,
                                          {},
                                          source.events()));
    require(schedule.run().completed_cycles() == 1 &&
                schedule.run().hardware_reset_ticks() == recovery_end &&
                schedule.run().elapsed_ticks() == first_cycle &&
                schedule.run().timer_clk2_edges() == 0,
            "Post-recovery pipeline resumes one genuine45tick period and fresh CLK2 phase");
  }
}

void strict_tie() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  Source source{.ticks = {period}, .order = DspExternalOrder::reject_tie};
  const auto cpu = machine.cpu().debug_state();
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 1},
                                    cycle_inputs,
                                    {},
                                    source.events()));
          }).contains("Unqualified simultaneous") &&
              schedule.faulted() && source.cursor == 0 && schedule.run().completed_cycles() == 0 &&
              machine.cpu().debug_state() == cpu,
          "Unqualified ties preserve native prefix and peripheral cursor while quarantining the "
          "schedule");
}
} // namespace
} // namespace blofeld::test::fetch_events

int main() {
  try {
    using namespace blofeld;
    using namespace blofeld::test::fetch_events;
    guards();
    pin_capture();
    ordered(DspExternalOrder::external_before_dsp);
    ordered(DspExternalOrder::dsp_before_external);
    boundary_and_replay();
    reset();
    strict_tie();
    std::cout
        << "Native pipeline periods and external event boundaries retain explicit ordering.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
