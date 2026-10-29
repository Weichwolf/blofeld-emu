// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned extal = 12000000, timebase = 540000000, nops = 64, budget = 4;
constexpr std::uint64_t period = 45, twice = 90, third = 135, future = 1000;
constexpr unsigned initial_status = 0xc00300;

void require(const bool condition, const std::string_view label) {
  if (!condition) { throw Fault(std::string(label)); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a visible peripheral scheduling refusal");
}

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

DspMachine &prepare(DspMachine &machine, const bool running) {
  test::DspProgram program;
  for (unsigned i = 0; i < nops; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  if (running) {
    machine.bus().pll().observe_pinit(false);
    machine.cpu().reset_spi_boot();
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  }
  return machine;
}

struct Sources {
  std::shared_ptr<const DspFetchEventIdentity> inherited_id =
      std::make_shared<const DspFetchEventIdentity>();
  std::shared_ptr<const PeripheralEventIdentity> peripheral_id =
      std::make_shared<const PeripheralEventIdentity>();
  std::optional<std::uint64_t> inherited;
  std::optional<std::uint64_t> peripheral = period;
  std::vector<std::string> log;
  unsigned external_calls = 0, peripheral_calls = 0;
  unsigned maximum = budget;

  DspFetchExternalEvents external() {
    return {.identity = inherited_id,
            .next_ticks = [this] { return inherited; },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(inherited == ticks, "Inherited input consumes its actual deadline");
                  log.push_back("E" + std::to_string(ticks));
                  inherited.reset();
                  ++external_calls;
                },
            .order = DspExternalOrder::external_before_dsp,
            .maximum_events = maximum};
  }

  DspFetchPeripheralEvents events() {
    return {.identity = peripheral_id,
            .next_ticks = [this] { return peripheral; },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(peripheral == ticks, "Peripheral consumes its actual deadline");
                  log.push_back("P" + std::to_string(ticks));
                  peripheral.reset();
                  ++peripheral_calls;
                },
            .order = DspExternalOrder::external_before_dsp,
            .maximum_events = maximum};
  }
};

unsigned drive(DspFetchScheduledRun &schedule,
               Sources &source,
               const std::uint64_t target,
               const unsigned cycles,
               const std::function<void()> &after = {}) {
  return schedule.advance_to(
      {.horizon = {.ticks = target}, .maximum_cycles = cycles},
      cycle_inputs,
      [&](const DspFetchCycleResult &result) {
        require(result.period_ticks == period, "Literal 12MHz native period remains 45 ticks");
        source.log.push_back("D" + std::to_string(result.completion_ticks));
        if (after) { after(); }
      },
      source.external(),
      source.events());
}

struct Rig {
  explicit Rig(const bool held = false)
      : schedule(
            inputs(),
            prepare(machine, !held),
            held ? std::optional{DspHardwareResetInputs{
                       .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}}
                 : std::nullopt) {}

  DspMachine machine;
  DspFetchScheduledRun schedule;
  Sources source;
};

void ordered() {
  Rig rig;
  rig.source.inherited = period;
  require(drive(rig.schedule, rig.source, twice, 2) == 2, "Two actual native cycles complete");
  const PeripheralCalendarState expected{.horizon = {.ticks = twice},
                                         .identity = rig.source.peripheral_id,
                                         .last_native = twice,
                                         .last_external = period,
                                         .last_peripheral = period,
                                         .causes = 4,
                                         .peripheral_events = 1,
                                         .advanced = true};
  require(rig.source.log == std::vector<std::string>{"E45", "P45", "D45", "D90"} &&
              rig.schedule.peripheral_state() == expected &&
              rig.schedule.state().external_events == 1 && rig.machine.cpu().state().a == 0 &&
              rig.machine.cpu().state().sr == initial_status &&
              rig.schedule.run().elapsed_ticks() == twice,
          "Independent three-actor order,whole calendar,flags and elapsed time match");
}

void causal() {
  constexpr std::uint64_t generated = 78;
  constexpr std::uint64_t peer_generated = 123;
  Rig rig;
  rig.source.peripheral = future;
  rig.source.inherited = future;
  static_cast<void>(drive(rig.schedule, rig.source, period - 1, 0));
  static_cast<void>(drive(rig.schedule, rig.source, twice, 2, [&] {
    if (rig.schedule.run().completed_cycles() == 1) { rig.source.peripheral = generated; }
  }));
  require(rig.source.log == std::vector<std::string>{"D45", "P78", "D90"} &&
              rig.source.inherited == future && rig.source.external_calls == 0,
          "Actual native completion reschedules only the separate peripheral lane");
  rig.source.peripheral = peer_generated;
  rig.schedule.observe_peripheral_peer(rig.source.events());
  static_cast<void>(drive(rig.schedule, rig.source, third, 1));
  const auto state = rig.schedule.peripheral_state();
  require(state && state->last_peer == twice && state->last_peripheral == peer_generated &&
              state->last_native == third && state->causes == 6 && state->peripheral_events == 2 &&
              rig.source.log == std::vector<std::string>{"D45", "P78", "D90", "P123", "D135"},
          "Accepted peer cause retains independent input and literal full causal counts");
}

void state() {
  Rig rig;
  rig.source.inherited = period;
  rig.source.peripheral = period + 1;
  static_cast<void>(
      rig.schedule.advance_to({.horizon = {.ticks = period, .native = DspScheduleBoundary::before}},
                              cycle_inputs,
                              {},
                              rig.source.external(),
                              rig.source.events()));
  const auto saved = rig.schedule.checkpoint(rig.source.events());
  const auto source = rig.source;
  const auto cpu = rig.machine.cpu().debug_state();
  const auto calendar = rig.schedule.peripheral_state();
  static_cast<void>(drive(rig.schedule, rig.source, twice, 2));
  const auto final_cpu = rig.machine.cpu().debug_state();
  const auto final_calendar = rig.schedule.peripheral_state();
  for (unsigned replay = 0; replay < 2; ++replay) {
    rig.source = source;
    rig.schedule.restore(saved, rig.source.events());
    require(rig.machine.cpu().debug_state() == cpu && rig.schedule.peripheral_state() == calendar,
            "Trusted restore retains complete native and calendar state at a partial boundary");
    static_cast<void>(drive(rig.schedule, rig.source, twice, 2));
    require(rig.machine.cpu().debug_state() == final_cpu &&
                rig.schedule.peripheral_state() == final_calendar,
            "Two real dependent suffixes preserve full CPU/stack and calendar state");
  }
  DspMachine target;
  auto cloned_source = source;
  DspFetchScheduledRun clone(saved, target, cloned_source.events());
  require(clone.peripheral_state() == calendar, "Distinct native clone retains the whole calendar");
  static_cast<void>(drive(clone, cloned_source, twice, 2));
  require(target.cpu().debug_state() == final_cpu && clone.peripheral_state() == final_calendar,
          "Trusted native/calendar clone executes the same independent native suffix");
  require(rejects([&] { clone.restore(saved, cloned_source.events()); }).contains("matching"),
          "Original snapshot cannot replace a distinct cloned owner");
}

void guards() {
  Rig rig;
  const auto initial = rig.schedule.checkpoint();
  const auto original = rig.source;
  const auto cpu = rig.machine.cpu().debug_state();
  const std::array<std::function<void(DspFetchPeripheralEvents &)>, 6> malformed{
      [](auto &e) { e.identity.reset(); },
      [](auto &e) { e.next_ticks = {}; },
      [](auto &e) { e.observe = {}; },
      [](auto &e) { e.maximum_events = 0; },
      [](auto &e) { e.order = DspExternalOrder::unqualified; },
      [](auto &e) { e.order = DspExternalOrder::dsp_before_external; }};
  for (const auto &change : malformed) {
    rig.schedule.restore(initial);
    rig.source = original;
    auto events = rig.source.events();
    change(events);
    require(rejects([&] {
              static_cast<void>(
                  rig.schedule.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 1},
                                          cycle_inputs,
                                          {},
                                          rig.source.external(),
                                          events));
            }).contains("require identity") &&
                rig.schedule.faulted() && rig.source.peripheral_calls == 0 &&
                rig.machine.cpu().debug_state() == cpu,
            "Malformed peripheral contracts fail before input/native work");
  }
  rig.schedule.restore(initial);
  static_cast<void>(drive(rig.schedule, rig.source, 1, 0));
  const auto bound = rig.schedule.checkpoint(rig.source.events());
  rig.source.peripheral = period + 1;
  require(rejects([&] {
            static_cast<void>(drive(rig.schedule, rig.source, period, 1));
          }).contains("pinned pending") &&
              rig.schedule.faulted() && rig.schedule.run().completed_cycles() == 0,
          "An arbitrary changed deadline cannot become a fabricated hardware cause");
  rig.source = original;
  rig.schedule.restore(bound, rig.source.events());
  rig.schedule.restore(initial);
  require(!rig.schedule.peripheral_state(),
          "An earlier unbound snapshot restores the actual contract");
  rig.schedule.restore(bound, rig.source.events());
  require(rig.schedule.peripheral_state().has_value(),
          "A trusted later bound snapshot survives recovery through an earlier unbound state");
}

void shared_budget() {
  Rig rig;
  rig.source.maximum = 1;
  rig.source.inherited = period;
  require(rejects([&] {
            static_cast<void>(drive(rig.schedule, rig.source, period, 1));
          }).contains("event budget exhausted") &&
              rig.schedule.faulted() && rig.source.external_calls == 1 &&
              rig.source.peripheral_calls == 0 && rig.schedule.run().completed_cycles() == 0,
          "Inherited/peripheral events share one unchanged admission budget");
  Rig changed;
  static_cast<void>(drive(changed.schedule, changed.source, 1, 0));
  auto events = changed.source.events();
  ++events.maximum_events;
  require(rejects([&] {
            static_cast<void>(changed.schedule.advance_to(
                {.horizon = {.ticks = period}}, {}, {}, changed.source.external(), events));
          }).contains("fixed event budget") &&
              changed.schedule.faulted(),
          "A bound peripheral budget cannot silently increase between horizons");
}

void query_guards() {
  Rig rig;
  auto events = rig.source.events();
  events.next_ticks = [&]() -> std::optional<std::uint64_t> {
    rig.machine.cpu().reset_spi_boot();
    return period;
  };
  require(!rejects([&] {
             static_cast<void>(
                 rig.schedule.advance_to({.horizon = {.ticks = 1}}, {}, {}, {}, events));
           }).empty() &&
              rig.schedule.faulted() && rig.schedule.run().completed_cycles() == 0,
          "Actual native reset mutation in a peripheral query fails visibly");
  Rig reentrant;
  auto recursive = reentrant.source.events();
  recursive.next_ticks = [&]() -> std::optional<std::uint64_t> {
    static_cast<void>(reentrant.schedule.checkpoint());
    return period;
  };
  require(!rejects([&] {
             static_cast<void>(
                 reentrant.schedule.advance_to({.horizon = {.ticks = 1}}, {}, {}, {}, recursive));
           }).empty() &&
              reentrant.schedule.faulted(),
          "Query cannot recursively capture or advance its native owner");
}

void reset() {
  constexpr std::uint64_t held = 100;
  constexpr std::uint64_t generated = 140;
  constexpr std::uint64_t recovery = 190;
  constexpr std::uint64_t first_native = 235;
  constexpr std::uint64_t early = 20;
  Rig rig(true);
  rig.source.inherited = early;
  rig.source.peripheral = early;
  require(drive(rig.schedule, rig.source, held, 0) == 0 &&
              rig.source.log == std::vector<std::string>{"E20", "P20"} &&
              rig.schedule.run().elapsed_ticks() == held &&
              rig.schedule.run().completed_cycles() == 0,
          "Both actual sources advance while native DSP reset is held");
  const auto snapshot = rig.schedule.checkpoint(rig.source.events());
  const auto source = rig.source;
  std::optional<DspDebugState> golden;
  for (unsigned replay = 0; replay < 2; ++replay) {
    rig.source = source;
    rig.schedule.restore(snapshot, rig.source.events());
    rig.source.peripheral = generated;
    rig.schedule.observe_peripheral_peer(rig.source.events());
    rig.schedule.release_hardware_reset(rig.source.events());
    require(drive(rig.schedule, rig.source, first_native, 1) == 1 &&
                rig.schedule.run().hardware_reset_ticks() == recovery &&
                rig.source.log == std::vector<std::string>{"E20", "P20", "P140", "D235"},
            "Literal held/recovery fractions and peripheral event precede the first native cycle");
    const auto now = rig.machine.cpu().debug_state();
    if (golden) {
      require(*golden == now, "Held native/calendar restore repeats complete CPU state");
    }
    golden = now;
  }
}
}

int main(const int argc, const char *const *argv) {
  try {
    require(argc == 2, "Choose order,causal,state,guards or reset");
    const std::string_view mode(argv[1]);
    if (mode == "order") {
      ordered();
    } else if (mode == "causal") {
      causal();
    } else if (mode == "state") {
      state();
    } else if (mode == "guards") {
      guards();
      shared_budget();
      query_guards();
    } else if (mode == "reset") {
      reset();
    } else {
      throw Fault("Unknown peripheral scope");
    }
    std::cout << "PASS native separate peripheral lane: " << mode << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
