// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_fetch_schedule_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::fetch_events {
namespace {
void malformed_sources() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  const auto saved = schedule.checkpoint();
  const auto cpu = machine.cpu().debug_state();
  Source source{.ticks = {period}};
  const std::array<std::function<void(DspFetchExternalEvents &)>, 6> corrupt{
      [](auto &events) { events.identity.reset(); },
      [](auto &events) { events.next_ticks = {}; },
      [](auto &events) { events.observe = {}; },
      [](auto &events) { events.maximum_events = 0; },
      [](auto &events) { events.order = DspExternalOrder::unqualified; },
      [](auto &events) {
        events.order = std::bit_cast<DspExternalOrder>(std::uint8_t{event_budget});
      }};
  for (const auto &change : corrupt) {
    schedule.restore(saved);
    auto events = source.events();
    change(events);
    require(rejects([&] {
              static_cast<void>(
                  schedule.advance_to({.horizon = {.ticks = period}}, {}, {}, events));
            }).contains("require identity") &&
                schedule.faulted() && source.cursor == 0 && machine.cpu().debug_state() == cpu &&
                schedule.run().completed_cycles() == 0,
            "Malformed source contracts refuse before any native work or event observation");
  }
  schedule.restore(saved);
  static_cast<void>(schedule.advance_to({.horizon = {.ticks = 1}}, {}));
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = 2}}, {}, {}, source.events()));
          }).contains("bind on its first advance") &&
              schedule.faulted(),
          "A new stream cannot appear after the initial source contract");
}

void retained_source() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  Source source{.ticks = {1}};
  static_cast<void>(schedule.advance_to({.horizon = {.ticks = 1}}, {}, {}, source.events()));
  const auto saved = schedule.checkpoint();
  const std::array<std::function<void(DspFetchExternalEvents &)>, 3> corrupt{
      [](auto &events) { events = {}; },
      [](auto &events) { events.identity = std::make_shared<const DspFetchEventIdentity>(); },
      [](auto &events) { events.order = DspExternalOrder::dsp_before_external; }};
  for (const auto &change : corrupt) {
    schedule.restore(saved);
    auto events = source.events();
    change(events);
    require(rejects([&] {
              static_cast<void>(schedule.advance_to({.horizon = {.ticks = 2}}, {}, {}, events));
            }).contains("source") &&
                schedule.faulted() && source.cursor == 1,
            "Exhaustion does not discard the source identity, order or continued binding");
  }
  schedule.restore(saved);
  source.ticks.push_back(1);
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = 2}}, {}, {}, source.events()));
          }).contains("consumed timeline") &&
              schedule.faulted() && source.cursor == 1,
          "An exhausted stream cannot append into an already consumed external boundary");
}

void pinned_and_monotone() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  Source source{.ticks = {period}};
  static_cast<void>(
      schedule.advance_to({.horizon = {.ticks = period - 1}}, {}, {}, source.events()));
  source.ticks.front() = period + 1;
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 1},
                                    cycle_inputs,
                                    {},
                                    source.events()));
          }).contains("pinned pending deadline") &&
              source.cursor == 0 && schedule.run().completed_cycles() == 0 && schedule.faulted(),
          "A previously announced pending edge cannot move when the caller resumes");
}

void causal_append() {
  for (const auto order : {DspExternalOrder::external_before_dsp,
                           DspExternalOrder::dsp_before_external,
                           DspExternalOrder::reject_tie}) {
    DspMachine machine;
    DspFetchScheduledRun schedule(inputs(), prepare(machine));
    Source source{.ticks = {}, .order = order};
    const auto saved = schedule.checkpoint();
    const auto append = [&](const DspFetchCycleResult &) { source.ticks.push_back(period); };
    const auto advance = [&] {
      return schedule.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 1},
                                 cycle_inputs,
                                 append,
                                 source.events());
    };
    if (order == DspExternalOrder::dsp_before_external) {
      require(advance() == 1 && source.cursor == 1 && !schedule.faulted(),
              "A genuine completion may cause a same-tick event ordered after that completion");
    } else {
      require(rejects(advance).contains("consumed timeline") && schedule.faulted() &&
                  source.cursor == 0 && schedule.run().completed_cycles() == 1,
              "A newborn event cannot retroactively precede its actual native cause");
    }
    schedule.restore(saved);
    source.cursor = 0;
    source.ticks.clear();
    require(rejects([&] {
              static_cast<void>(schedule.advance_to(
                  {.horizon = {.ticks = period}, .maximum_cycles = 1},
                  cycle_inputs,
                  [&](const DspFetchCycleResult &) { source.ticks.push_back(period - 1); },
                  source.events()));
            }).contains("consumed timeline") &&
                source.cursor == 0 && schedule.run().completed_cycles() == 1 && schedule.faulted(),
            "No source may introduce an event earlier than completed native work");
  }
}

void callback_failures() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  const auto saved = schedule.checkpoint();
  Source source{.ticks = {1}};
  auto events = source.events();
  events.next_ticks = [&]() -> std::optional<std::uint64_t> {
    machine.cpu().reset_spi_boot();
    return 1;
  };
  require(rejects([&] {
            static_cast<void>(schedule.advance_to({.horizon = {.ticks = 1}}, {}, {}, events));
          }).contains("coordinated") &&
              schedule.faulted() && source.cursor == 0 && schedule.run().completed_cycles() == 0,
          "A query cannot change the native owner/front end");
  schedule.restore(saved);
  events = source.events();
  events.observe = [&](std::uint64_t) { static_cast<void>(schedule.checkpoint()); };
  require(rejects([&] {
            static_cast<void>(schedule.advance_to({.horizon = {.ticks = 1}}, {}, {}, events));
          }).contains("inactive owner") &&
              schedule.faulted() && source.cursor == 0 && schedule.run().completed_cycles() == 0,
          "An observation cannot reenter the active timeline checkpoint interface");
  schedule.restore(saved);
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 1}, [&] {
                  machine.cpu().reset_spi_boot();
                  return cycle_inputs();
                }));
          }).contains("coordinated") &&
              schedule.faulted() && schedule.run().completed_cycles() == 0,
          "The cycle input provider cannot mutate native ownership before admission");
}

void budgets() {
  DspMachine machine;
  DspFetchScheduledRun schedule(inputs(), prepare(machine));
  const auto saved = schedule.checkpoint();
  Source source{.ticks = {1, period + 1}};
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = 2 * period}, .maximum_cycles = 1},
                                    cycle_inputs,
                                    {},
                                    source.events()));
          }).contains("cycle budget") &&
              schedule.run().completed_cycles() == 1 && schedule.state().external_events == 2 &&
              source.cursor == 2 && schedule.faulted(),
          "Budget refusal retains exactly the actual committed native and event prefix");
  schedule.restore(saved);
  source.cursor = 0;
  source.ticks = {period, period};
  auto events = source.events();
  events.maximum_events = 1;
  require(rejects([&] {
            static_cast<void>(schedule.advance_to(
                {.horizon = {.ticks = period}, .maximum_cycles = 1}, cycle_inputs, {}, events));
          }).contains("event budget") &&
              schedule.run().completed_cycles() == 0 && source.cursor == 1 &&
              schedule.state().external_events == 1 && schedule.faulted(),
          "Finite event budgets bound a same-timestamp burst without discarding its order");
  schedule.restore(saved);
  source.cursor = 0;
  require(schedule.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 1},
                              cycle_inputs,
                              {},
                              source.events()) == 1 &&
              source.cursor == 2 && !schedule.faulted(),
          "Restoring the trusted owner and cursor resumes both same-timestamp pin edges");
  schedule.restore(saved);
  source.cursor = 0;
  source.ticks = {2, 1};
  require(rejects([&] {
            static_cast<void>(
                schedule.advance_to({.horizon = {.ticks = period}}, {}, {}, source.events()));
          }).contains("moved backwards") &&
              source.cursor == 1 && schedule.run().completed_cycles() == 0 && schedule.faulted(),
          "A backwards stream fails immediately after its actual first observation");
}

void absolute_epoch_and_horizons() {
  constexpr std::uint64_t epoch = 1000;
  DspMachine machine;
  auto contract = inputs();
  contract.epoch_ticks = epoch;
  DspFetchScheduledRun schedule(contract, prepare(machine));
  Source source{.ticks = {epoch, epoch + period}, .order = DspExternalOrder::dsp_before_external};
  require(schedule.advance_to({.horizon = {.ticks = epoch + period}, .maximum_cycles = 1},
                              cycle_inputs,
                              {},
                              source.events()) == 1 &&
              schedule.run().elapsed_ticks() == period &&
              schedule.state().last_native == epoch + period && source.cursor == 2,
          "Absolute event epochs retain relative native period accounting without shifting either");
  const auto state = schedule.state();
  const auto cpu = machine.cpu().debug_state();
  for (const auto boundary :
       {DspFetchHorizon{.ticks = epoch + period - 1},
        DspFetchHorizon{.ticks = epoch + period, .native = DspScheduleBoundary::before},
        DspFetchHorizon{.ticks = epoch + period, .external = DspScheduleBoundary::before}}) {
    require(
        rejects([&] {
          static_cast<void>(schedule.advance_to({.horizon = boundary}, {}, {}, source.events()));
        }).contains("cannot rewind") &&
            schedule.state() == state && machine.cpu().debug_state() == cpu && !schedule.faulted(),
        "Invalid rewind requests refuse before disturbing a healthy consumed boundary");
  }
  DspMachine other;
  DspFetchScheduledRun foreign(contract, prepare(other));
  const auto other_cpu = other.cpu().debug_state();
  const auto saved = schedule.checkpoint();
  require(
      rejects([&] { foreign.restore(saved); }).contains("matching inactive owner") &&
          other.cpu().debug_state() == other_cpu && !foreign.faulted(),
      "A trusted snapshot cannot transplant native state or a source cursor into another owner");
}

void mixed_horizons() {
  for (const auto order :
       {DspExternalOrder::external_before_dsp, DspExternalOrder::dsp_before_external}) {
    DspMachine machine;
    DspFetchScheduledRun schedule(inputs(), prepare(machine));
    Source source{.ticks = {period}, .order = order};
    const auto cpu = machine.cpu().debug_state();
    DspFetchHorizon cut{.ticks = period};
    if (order == DspExternalOrder::external_before_dsp) {
      cut.external = DspScheduleBoundary::before;
    } else {
      cut.native = DspScheduleBoundary::before;
    }
    require(rejects([&] {
              static_cast<void>(schedule.advance_to(
                  {.horizon = cut, .maximum_cycles = 1}, cycle_inputs, {}, source.events()));
            }).contains("contradicts") &&
                source.cursor == 0 && schedule.faulted() && machine.cpu().debug_state() == cpu &&
                schedule.run().completed_cycles() == 0,
            "An inconsistent mixed horizon cannot execute the later source before its cause");
  }
}
} // namespace

void guards() {
  malformed_sources();
  retained_source();
  pinned_and_monotone();
  causal_append();
  callback_failures();
  budgets();
  mixed_horizons();
  absolute_epoch_and_horizons();
}
} // namespace blofeld::test::fetch_events
