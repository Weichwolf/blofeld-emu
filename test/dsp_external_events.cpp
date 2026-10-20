// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "dsp_external_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::external {
namespace {
void boundaries_and_checkpoint() {
  Fixture f;
  auto source = f.source();
  constexpr unsigned end = 1000;
  constexpr unsigned completed = 8;
  require(f.runner.advance_to(f.machine, bit_half, before, completed, {}, {}, source) == 5 &&
              f.applied == std::vector<std::uint64_t>{0, partial} &&
              f.runner.next_event_ticks(f.machine) == bit_half &&
              f.runner.next_completion_ticks(f.machine) == 6 * unit_ticks,
          "Non-grid events run between native instructions and respect exclusive boundaries");
  const auto cpu = f.machine.checkpoint();
  const auto schedule = f.runner.checkpoint(f.machine);
  const auto cursor = f.cursor;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.machine.restore(cpu);
    f.runner.restore(schedule, f.machine);
    f.cursor = cursor;
    f.applied.clear();
    require(f.runner.advance_to(f.machine, bit_half, through, 0, {}, {}, source) == 0 &&
                f.applied == std::vector<std::uint64_t>{bit_half} &&
                f.runner.completed_external_events() == 3,
            "A restored exclusive deadline applies once without charging a native instruction");
    require(f.runner.advance_to(f.machine, end, through, completed, {}, {}, source) == 3 &&
                f.machine.run_clock().elapsed_ticks() == completed * unit_ticks &&
                f.machine.run_clock().native_cycle_units() == completed &&
                f.runner.next_event_ticks(f.machine) == (completed + 1) * unit_ticks &&
                f.runner.completed_steps() == completed,
            "Exhausted external sources keep the independent native timeline and pending plan");
    require(f.runner.advance_to(f.machine, end, through, 0, {}, {}, source) == 0 &&
                f.applied.size() == 1,
            "Repeated horizons neither repeat events nor synthesize DSP cycles");
  }
}

void exact_ties() {
  for (const auto order : {DspExternalOrder::reject_tie,
                           DspExternalOrder::dsp_before_external,
                           DspExternalOrder::external_before_dsp}) {
    Fixture f;
    constexpr unsigned tie = 2 * unit_ticks;
    f.deadlines = {partial, tie};
    auto source = f.source(order);
    std::vector<char> ledger;
    source.observe = [&](const std::uint64_t ticks) {
      ledger.push_back('E');
      const unsigned expected =
          ticks == partial || order == DspExternalOrder::external_before_dsp ? 1U : 2U;
      require(f.runner.completed_steps() == expected &&
                  f.machine.run_clock().native_cycle_units() == expected,
              "The explicitly selected tie order determines the completed native prefix");
      ++f.cursor;
    };
    const DspCompletionObserver after{
        .observe = [&](const auto & /*step*/) { ledger.push_back('D'); }};
    require(f.runner.advance_to(f.machine, tie, before, 2, {}, after, source) == 1 &&
                ledger == std::vector<char>{'D', 'E'},
            "Both simultaneous events remain pending at an exclusive boundary");
    if (order == DspExternalOrder::reject_tie) {
      const auto error =
          rejects([&] { f.runner.advance_to(f.machine, tie, through, 2, {}, after, source); });
      require(error.contains("simultaneous external") && f.runner.completed_steps() == 1 &&
                  f.runner.completed_external_events() == 1 && f.runner.faulted(),
              "An unqualified tie rejects before either tied effect");
    } else {
      require(f.runner.advance_to(f.machine, tie, through, 2, {}, after, source) == 1,
              "A qualified tie completes one real native instruction");
      const std::vector<char> expected = order == DspExternalOrder::dsp_before_external
                                             ? std::vector<char>{'D', 'E', 'D', 'E'}
                                             : std::vector<char>{'D', 'E', 'E', 'D'};
      require(
          ledger == expected && f.cursor == 2,
          "Native completion observations and external events have an independent literal order");
    }
  }
}

void change_source(const unsigned scenario, Fixture &f, DspExternalEvents &source) {
  if (scenario == 3) { source = {}; }
  if (scenario == 4) { ++f.deadlines.at(f.cursor); }
  if (scenario == 5) { source.order = DspExternalOrder::dsp_before_external; }
  if (scenario == 6) {
    source.next_ticks = [] { return std::nullopt; };
  }
  if (scenario == 7) { f.deadlines.at(f.cursor) = 0; }
}

void source_contract() {
  constexpr unsigned bound = 1;
  for (unsigned scenario = 0; scenario < 8; ++scenario) {
    Fixture f;
    auto source = f.source();
    if (scenario == 0) { source.observe = {}; }
    if (scenario == 1) { source.maximum_events = 0; }
    if (scenario == 2) { source.order = DspExternalOrder::unqualified; }
    if (scenario < 3) {
      static_cast<void>(
          rejects([&] { f.runner.advance_to(f.machine, bound, through, 1, {}, {}, source); }));
    } else {
      f.runner.advance_to(f.machine, bound, through, 1, {}, {}, source);
      change_source(scenario, f, source);
      static_cast<void>(
          rejects([&] { f.runner.advance_to(f.machine, partial, through, 2, {}, {}, source); }));
    }
    require(f.runner.faulted() && f.machine.run_clock().elapsed_ticks() == 0,
            "Missing, changed or invalid external contracts reject without executing the DSP");
  }
  Fixture late;
  late.runner.advance_to(late.machine, bound, through, 1);
  auto source = late.source();
  require(rejects([&] {
            late.runner.advance_to(late.machine, partial, through, 2, {}, {}, source);
          }).contains("first advance"),
          "A source cannot retroactively inject events into an already advanced timeline");
}

void failures_and_recovery() {
  for (unsigned scenario = 0; scenario < 4; ++scenario) {
    Fixture f;
    f.deadlines = {partial, partial + 1};
    auto source = f.source();
    const auto cpu = f.machine.checkpoint();
    const auto schedule = f.runner.checkpoint(f.machine);
    if (scenario == 0) { source.maximum_events = 1; }
    if (scenario == 1) {
      source.observe = [](const auto /*ticks*/) { throw Fault("Own external signal failure"); };
    }
    if (scenario == 2) { f.deadlines.back() = partial; }
    if (scenario == 3) {
      source.observe = [&](const auto /*ticks*/) { f.machine.cpu().step(); };
    }
    constexpr unsigned horizon = 200;
    const auto error =
        rejects([&] { f.runner.advance_to(f.machine, horizon, through, 2, {}, {}, source); });
    const bool completed_event = scenario == 0 || scenario == 2;
    require(f.runner.faulted() && f.runner.completed_steps() == 1 &&
                f.runner.completed_external_events() == (completed_event ? 1U : 0U) &&
                f.machine.run_clock().elapsed_ticks() == (scenario == 3 ? 2U : 1U) * unit_ticks &&
                error.contains("pc=") && error.contains("attempted-external=") &&
                error.contains("completed-external-events="),
            "Failures preserve explicit native/external prefixes and quarantine partial effects");
    f.machine.restore(cpu);
    f.runner.restore(schedule, f.machine);
    f.cursor = 0;
    f.applied.clear();
    f.deadlines = {partial, partial + 1};
    source = f.source();
    require(f.runner.advance_to(f.machine, horizon, through, 2, {}, {}, source) == 1 &&
                f.applied == f.deadlines && f.runner.completed_external_events() == 2,
            "Coordinated native/schedule/source restore replays both external edges exactly");
  }
}

void query_and_reentry_guards() {
  Fixture f;
  auto source = f.source();
  source.next_ticks = [&]() -> std::optional<std::uint64_t> {
    f.machine.cpu().step();
    return partial;
  };
  require(rejects([&] {
            f.runner.advance_to(f.machine, partial, through, 2, {}, {}, source);
          }).contains("coordinated recovery") &&
              f.runner.faulted(),
          "An event query cannot secretly advance the native clock");
  Fixture guarded;
  auto valid = guarded.source();
  valid.observe = [&](const auto /*ticks*/) {
    static_cast<void>(
        rejects([&] { static_cast<void>(guarded.runner.checkpoint(guarded.machine)); }));
    static_cast<void>(rejects(
        [&] { guarded.runner.advance_to(guarded.machine, partial, through, 2, {}, {}, valid); }));
    ++guarded.cursor;
  };
  require(guarded.runner.advance_to(guarded.machine, partial, through, 2, {}, {}, valid) == 1 &&
              guarded.runner.completed_external_events() == 2,
          "Active external observations cannot reenter or capture their schedule");
}

void completion_cannot_move_source() {
  for (const bool exhausted : {false, true}) {
    Fixture f;
    f.deadlines = exhausted ? std::vector<std::uint64_t>{0} : std::vector<std::uint64_t>{partial};
    const auto source = f.source();
    const DspCompletionObserver after{.observe = [&](const auto & /*step*/) {
      if (exhausted) {
        f.deadlines.push_back(partial);
      } else {
        ++f.deadlines.front();
      }
    }};
    const auto error =
        rejects([&] { f.runner.advance_to(f.machine, partial, through, 2, {}, after, source); });
    require(error.contains("pinned deadline") && f.runner.completed_steps() == 1 &&
                f.runner.completed_external_events() == (exhausted ? 1U : 0U) &&
                f.machine.run_clock().elapsed_ticks() == unit_ticks,
            "Native callbacks cannot move a future event or resurrect an exhausted source");
  }
}
}
}

int main() {
  using namespace blofeld::test::external;
  try {
    boundaries_and_checkpoint();
    exact_ties();
    source_contract();
    failures_and_recovery();
    query_and_reentry_guards();
    completion_cannot_move_source();
    native_esai();
    paired_events();
    std::cout << "External event ordering, native ESAI and coordinated replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
