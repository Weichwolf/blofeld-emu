// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "fetch_clone_fixture.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {
using namespace clone_test;
constexpr unsigned suffix_periods = 24;

CycleFrame advance(OwnDspTimeline &timeline, OwnDspMachine &dsp) {
  auto input = pins();
  input.cycle = dsp.clock().native_cycle_units();
  const auto result =
      timeline.advance_one(timeline.next_deadline(), OwnDspBoundary::through, input);
  if (!result) { throw Fault("The actual next reference period must complete"); }
  return observe(*result, dsp);
}

void test_native_clones() {
  for (unsigned cut = 0; cut < 10; ++cut) {
    OwnDspMachine source(dsp_clock);
    prepare(source);
    OwnDspFetch live(source);
    for (unsigned n = 0; n < cut; ++n) { static_cast<void>(clone_test::advance(live, source)); }
    const auto saved = live.checkpoint();
    const auto before = frame(source, &live);
    std::vector<CycleFrame> expected;
    expected.reserve(suffix_periods);
    for (unsigned n = 0; n < suffix_periods; ++n) {
      expected.push_back(clone_test::advance(live, source));
    }
    const auto source_end = frame(source, &live);
    require(source_end.progress.instructions >= 3,
            "The fixed suffix must reach real MOVE,CLR and independently checked ADD retirements");
    for (unsigned replay = 0; replay < 2; ++replay) {
      OwnDspMachine target(dsp_clock);
      OwnDspFetch copied(target, saved);
      require(frame(target, &copied) == before && frame(source, &live) == source_end &&
                  target.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word,
              "Every native F1/F2/effect cut transfers full CPU,fetch continuation and known RAM");
      const auto own = copied.checkpoint();
      std::vector<CycleFrame> observed;
      observed.reserve(suffix_periods);
      for (unsigned n = 0; n < suffix_periods; ++n) {
        observed.push_back(clone_test::advance(copied, target));
      }
      require(observed == expected && frame(source, &live) == source_end,
              "All24actual cloned periods equal the source with independent MOVE/CLR/ADD results");
      const OwnDspFetch restored(target, own);
      require(frame(target, &restored) == before &&
                  rejects([&] { static_cast<void>(copied.view()); }).contains("Stale"),
              "Explicit same-chip restoration retains the cut and revokes the old fetch owner");
    }
  }
}

void test_excluded_boundary() {
  OwnDspMachine source(dsp_clock);
  prepare(source);
  OwnDspTimeline live(source, timing, 0);
  const auto deadline = live.next_deadline();
  require(deadline == period * 5 && source.clock().hardware_reset_ticks() == period * 4 &&
              !live.advance_one(deadline, OwnDspBoundary::before, pins()) &&
              source.clock().native_cycle_units() == 0,
          "An excluded same-time reference period cannot publish F1 or advance clocks");
  const auto saved = live.checkpoint();
  const auto before = frame(source);
  const auto expected = advance(live, source);
  OwnDspMachine target(dsp_clock);
  OwnDspTimeline copied(target, saved);
  require(frame(target) == before && copied.next_deadline() == deadline,
          "Timeline clone retains the excluded boundary and exact pending deadline");
  require(advance(copied, target) == expected && frame(target) == frame(source),
          "Including the saved boundary executes precisely the same real F1 period");
  require(rejects([&] {
            static_cast<void>(copied.advance_one(deadline, OwnDspBoundary::before, pins()));
          }).contains("rewind a consumed boundary"),
          "A consumed boundary cannot become excluded after cloning");
  static_cast<void>(advance(copied, target));
  require(target.clock().native_cycle_units() == 2,
          "A refused rewind preserves the next actual F2 continuation");
}

void test_timeline_address_reuse() {
  std::optional<OwnDspMachine> machine;
  std::optional<OwnDspTimeline> schedule;
  machine.emplace(dsp_clock);
  prepare(*machine);
  schedule.emplace(*machine, timing, 0);
  static_cast<void>(advance(*schedule, *machine));
  const auto saved = schedule->checkpoint();
  const auto expected = frame(*machine);
  const auto *const address = std::addressof(*machine);
  schedule.reset();
  machine.reset();
  machine.emplace(dsp_clock);
  require(address == std::addressof(*machine),
          "Controlled timeline storage reuses its exact address");
  schedule.emplace(*machine, saved);
  require(frame(*machine) == expected, "Transferred timeline retains the expired source state");
  const auto own = schedule->checkpoint();
  schedule.reset();
  schedule.emplace(*machine, own);
  static_cast<void>(advance(*schedule, *machine));
  require(machine->clock().native_cycle_units() == 2,
          "The new timeline advances the transferred genuine next F2 period");
}

void test_lower_address_reuse() {
  std::optional<OwnDspMachine> machine;
  std::optional<OwnDspFetch> fetch;
  machine.emplace(dsp_clock);
  prepare(*machine);
  fetch.emplace(*machine);
  static_cast<void>(clone_test::advance(*fetch, *machine));
  const auto saved = fetch->checkpoint();
  const auto expected = frame(*machine, &*fetch);
  const auto *const address = std::addressof(*machine);
  fetch.reset();
  machine.reset();
  machine.emplace(dsp_clock);
  require(address == std::addressof(*machine), "Controlled fetch storage reuses its exact address");
  fetch.emplace(*machine, saved);
  require(frame(*machine, &*fetch) == expected,
          "Explicit transfer survives source destruction and complete object-address reuse");
  const auto own = fetch->checkpoint();
  fetch.reset();
  fetch.emplace(*machine, own);
  static_cast<void>(clone_test::advance(*fetch, *machine));
}
}

int main() {
  try {
    test_native_clones();
    test_excluded_boundary();
    test_lower_address_reuse();
    test_timeline_address_reuse();
    std::cout << "DSP clones:10native cuts,20actual24-period suffixes,excluded boundary,address "
                 "reuse and stale fetch owners passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
