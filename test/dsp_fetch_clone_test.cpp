// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "fetch_clone_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {
using namespace clone_test;
constexpr unsigned suffix_periods = 24;

void test_native_clones() {
  for (unsigned cut = 0; cut < 10; ++cut) {
    DspMachine source;
    DspFetchRun live(dsp_inputs(), prepare(source));
    for (unsigned n = 0; n < cut; ++n) { independent(live.advance_cycle(source, pins()), source); }
    const auto saved = live.checkpoint(source);
    const auto before = frame(source, live);
    std::vector<Frame> expected;
    for (unsigned n = 0; n < suffix_periods; ++n) {
      independent(live.advance_cycle(source, pins()), source);
      expected.push_back(frame(source, live));
    }
    const auto source_end = frame(source, live);
    for (unsigned replay = 0; replay < 2; ++replay) {
      DspMachine target;
      DspFetchRun copied(saved, target);
      require(frame(target, copied) == before && frame(source, live) == source_end &&
                  target.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word,
              "Every early fetch/decode/AG/E1/E2 cut transfers full native state and known RAM");
      require(rejects([&] { copied.restore(saved, target); }).contains("matching inactive"),
              "A transferred snapshot never becomes an in-place restore token for a different run");
      const auto own = copied.checkpoint(target);
      std::vector<Frame> observed;
      for (unsigned n = 0; n < suffix_periods; ++n) {
        independent(copied.advance_cycle(target, pins()), target);
        observed.push_back(frame(target, copied));
      }
      require(observed == expected && frame(source, live) == source_end,
              "All24actual cloned pipeline periods equal the source and retain independent ADD "
              "results");
      copied.restore(own, target);
      require(frame(target, copied) == before,
              "The clone's own native checkpoint remains restorable");
    }
    const auto state = frame(source, live);
    require(rejects([&] { const DspFetchRun bad(saved, source); }).contains("distinct isolated") &&
                frame(source, live) == state,
            "A live original backend cannot be silently overwritten by cloning itself");
  }
}

void test_excluded_external_boundary() {
  DspMachine source;
  DspFetchScheduledRun live({.dsp = dsp_inputs()}, prepare(source));
  Cursor cursor;
  cursor.position = 1; // The declared next event is45, without an event at zero.
  auto events = cursor.events();
  events.order = DspExternalOrder::dsp_before_external;
  live.advance_to(
      {.horizon = {.ticks = period, .external = DspScheduleBoundary::before}, .maximum_cycles = 4},
      pins,
      {},
      events);
  require(live.run().completed_cycles() == 1 && cursor.observed.empty() &&
              live.state().next_external == period,
          "The cut contains one real F1 period and an explicitly excluded same-time event");
  const auto saved = live.checkpoint();
  const auto before = frame(source, live.run());
  const auto timeline = live.state();
  const auto queries = cursor.queries;
  cursor.forbidden = true;
  DspMachine target;
  DspFetchScheduledRun copied(saved, target);
  require(
      frame(target, copied.run()) == before && copied.state() == timeline &&
          cursor.queries == queries && cursor.observed.empty(),
      "Timeline clone retains excluded boundaries and source identity without external queries");
  require(rejects([&] { copied.restore(saved); }).contains("matching inactive"),
          "Lower timeline in-place restore remains owner-bound");
  cursor.forbidden = false;
  copied.advance_to({.horizon = {.ticks = period}, .maximum_cycles = 4}, pins, {}, events);
  require(copied.run().completed_cycles() == 1 &&
              cursor.observed == std::vector<std::uint64_t>{period},
          "Advancing the same horizon consumes exactly the pending event and no native period");
  const auto own = copied.checkpoint();
  copied.advance_to({.horizon = {.ticks = period * 2}, .maximum_cycles = 4}, pins, {}, events);
  cursor.position = 2;
  copied.restore(own);
  require(copied.run().completed_cycles() == 1 && copied.state().external_events == 1,
          "The copied timeline's own checkpoint preserves its event count");
}

void test_timeline_address_reuse() {
  std::optional<DspMachine> machine;
  std::optional<DspFetchScheduledRun> schedule;
  machine.emplace();
  schedule.emplace(DspFetchScheduleInputs{.dsp = dsp_inputs()}, prepare(*machine));
  schedule->advance_to({.horizon = {.ticks = period}, .maximum_cycles = 4}, pins);
  const auto saved = schedule->checkpoint();
  const auto expected = frame(*machine, schedule->run());
  const auto *const address = std::addressof(*machine);
  schedule.reset();
  machine.reset();
  machine.emplace();
  require(address == std::addressof(*machine),
          "Timeline target reuses the exact native backend address");
  schedule.emplace(saved, *machine);
  require(frame(*machine, schedule->run()) == expected &&
              rejects([&] { schedule->restore(saved); }).contains("matching inactive"),
          "Expired timeline identity cannot pass same-address in-place restore");
  const auto own = schedule->checkpoint();
  schedule->restore(own);
  schedule->advance_to({.horizon = {.ticks = period * 2}, .maximum_cycles = 4}, pins);
  require(schedule->run().completed_cycles() == 2,
          "The explicitly transferred timeline advances its real next F2 period");
}

void test_lower_address_reuse() {
  std::optional<DspMachine> machine;
  std::optional<DspFetchRun> run;
  machine.emplace();
  run.emplace(dsp_inputs(), prepare(*machine));
  independent(run->advance_cycle(*machine, pins()), *machine);
  const auto snapshot = run->checkpoint(*machine);
  const auto expected = frame(*machine, *run);
  const auto *const address = std::addressof(*machine);
  run.reset();
  machine.reset();
  machine.emplace();
  require(address == std::addressof(*machine),
          "Controlled native backend storage reuses the exact address");
  run.emplace(snapshot, *machine);
  require(
      frame(*machine, *run) == expected &&
          rejects([&] { run->restore(snapshot, *machine); }).contains("matching inactive"),
      "Expired run identity distinguishes explicit transfer from accidental address-based restore");
  const auto owned = run->checkpoint(*machine);
  run->restore(owned, *machine);
  independent(run->advance_cycle(*machine, pins()), *machine);
}
}

int main() {
  try {
    test_native_clones();
    test_excluded_external_boundary();
    test_lower_address_reuse();
    test_timeline_address_reuse();
    std::cout << "Lower clone:10pipeline cuts,20actual24-period suffixes,excluded input and "
                 "address reuse passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
