// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_schedule_fixture.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace blofeld::test::fetch_events {
namespace {
void malformed_sources() {
  const std::array<std::function<void(DspRuntimeEvents &)>, 8> corrupt{
      [](auto &events) { events.identity.reset(); },
      [](auto &events) { events.next_ticks = {}; },
      [](auto &events) { events.observe = {}; },
      [](auto &events) { events.maximum_events_per_period = 0; },
      [](auto &events) { events.order = DspPeerOrder::unqualified; },
      [](auto &events) { events.order = std::bit_cast<DspPeerOrder>(std::uint8_t{event_budget}); },
      [](auto &events) {
        events = {};
        events.order = DspPeerOrder::peer_first;
      },
      [](auto &events) {
        events = {};
        events.maximum_events_per_period = 1;
      }};
  Fixture fixture(DspPeerOrder::peer_first, {}, {}, false);
  const auto registers = fixture.dsp.cpu().registers();
  for (const auto &change : corrupt) {
    auto events = fixture.source.events();
    change(events);
    require(rejects([&] {
              const McuDspRuntime rejected(inputs(), fixture.mcu, fixture.dsp, events);
            }).contains("coherent event ownership") &&
                fixture.source.cursor == 0 && fixture.dsp.cpu().registers() == registers &&
                fixture.dsp.clock().elapsed_ticks() == 0,
            "Malformed fixed event contract changed native state before refusal");
  }
  auto contract = inputs();
  contract.epoch_ticks = 1;
  require(rejects([&] { fixture.connect(contract); }).contains("requires held reset"),
          "Arbitrary DSP epoch was accepted alongside an absolute native MCU clock");
}

void retained_source() {
  Fixture fixture(DspPeerOrder::peer_first, {1});
  const auto saved = fixture.runtime->checkpoint();
  const std::array<std::function<void(DspRuntimeEvents &)>, 4> corrupt{
      [](auto &events) { events = {}; },
      [](auto &events) { events.identity = std::make_shared<const bool>(true); },
      [](auto &events) { events.order = DspPeerOrder::dsp_first; },
      [](auto &events) { ++events.maximum_events_per_period; }};
  for (const auto &change : corrupt) {
    auto events = fixture.source.events();
    change(events);
    const auto registers = fixture.dsp.cpu().registers();
    require(rejects([&] {
              const McuDspRuntime rejected(saved, fixture.mcu, fixture.dsp, events);
            }).contains("matching external identity") &&
                fixture.dsp.cpu().registers() == registers && !fixture.runtime->faulted(),
            "Clone changed its bound source contract or mutated the healthy destination");
  }
  fixture.source.ticks.push_back(fixture.runtime->end_ticks());
  require(rejects([&] { fixture.step(); }).contains("consumed timeline") &&
              fixture.source.cursor == 1 && fixture.runtime->faulted(),
          "An exhausted stream appended into an already consumed external boundary");
}

void pinned_and_monotone() {
  Fixture fixture(DspPeerOrder::peer_first, {first_cycle});
  fixture.step();
  fixture.source.ticks.front() = first_cycle + 1;
  require(rejects([&] { fixture.step(); }).contains("pinned pending deadline") &&
              fixture.source.cursor == 0 && fixture.runtime->completed_cycles() == 0 &&
              fixture.runtime->faulted(),
          "An announced pending edge moved across genuine native MCU periods");
  Fixture backwards(DspPeerOrder::peer_first, {first_cycle - 2, first_cycle - 3});
  backwards.step();
  require(rejects([&] { backwards.step(); }).contains("moved backwards") &&
              backwards.source.cursor == 1 && backwards.runtime->completed_cycles() == 0 &&
              backwards.runtime->faulted(),
          "A backwards stream failed to preserve exactly its first real observation");
}

void causal_append() {
  constexpr auto second_cycle = first_cycle + period;
  for (const auto order :
       {DspPeerOrder::peer_first, DspPeerOrder::dsp_first, DspPeerOrder::reject_tie}) {
    Fixture fixture(order);
    fixture.on_cycle = [&](const DspRuntimeCycle &cycle) {
      if (cycle.completion_ticks == second_cycle) { fixture.source.ticks.push_back(second_cycle); }
    };
    fixture.run(2);
    if (order == DspPeerOrder::dsp_first) {
      fixture.step();
      require(fixture.source.cursor == 1 && fixture.runtime->completed_cycles() == 3 &&
                  fixture.source.log == std::vector<std::string>{"D630", "D675", "E675", "D720"},
              "Same-tick causal input did not occur between genuine DSP completions");
    } else {
      require(rejects([&] { fixture.step(); }).contains("consumed timeline") &&
                  fixture.runtime->faulted() && fixture.source.cursor == 0 &&
                  fixture.runtime->completed_cycles() == 2,
              "A newborn event retroactively preceded its actual native cause");
    }
    Fixture past(order);
    past.on_cycle = [&](const DspRuntimeCycle &cycle) {
      if (cycle.completion_ticks == second_cycle) { past.source.ticks.push_back(second_cycle - 1); }
    };
    past.run(2);
    require(rejects([&] { past.step(); }).contains("consumed timeline") &&
                past.runtime->completed_cycles() == 2 && past.source.cursor == 0 &&
                past.runtime->faulted(),
            "An earlier input crossed completed native work");
  }
  // This event is born in the first of two DSP periods within one MCU period.
  // It must be re-queried before the second DSP period, irrespective of tie order.
  for (const auto order : {DspPeerOrder::peer_first, DspPeerOrder::dsp_first}) {
    Fixture fixture(order);
    fixture.on_cycle = [&](const DspRuntimeCycle &cycle) {
      if (cycle.completion_ticks == second_cycle) {
        fixture.source.ticks.push_back(second_cycle + 1);
      }
    };
    fixture.run(3);
    require(fixture.source.log == std::vector<std::string>{"D630", "D675", "E676", "D720"},
            "A newly generated asynchronous event was batched past the next native cycle");
  }
}

void budgets() {
  Fixture cycles_limited(DspPeerOrder::peer_first, {first_cycle + period + 1}, {}, false);
  auto contract = inputs();
  contract.maximum_cycles_per_period = 1;
  cycles_limited.connect(contract);
  cycles_limited.run(2);
  require(rejects([&] { cycles_limited.step(); }).contains("period budget") &&
              cycles_limited.runtime->completed_cycles() == 2 &&
              cycles_limited.source.cursor == 1 && cycles_limited.runtime->faulted(),
          "Cycle budget lost the completed native/input prefix within an MCU period");
  Fixture limited(DspPeerOrder::peer_first, {first_cycle, first_cycle}, {}, false);
  limited.source.budget = 1;
  limited.connect(inputs());
  limited.step();
  require(rejects([&] { limited.step(); }).contains("period budget") &&
              limited.runtime->completed_cycles() == 0 && limited.source.cursor == 1 &&
              limited.runtime->faulted(),
          "Finite event budget did not bound a genuine same-timestamp input burst");
  Fixture burst(DspPeerOrder::peer_first, {first_cycle, first_cycle});
  burst.run(2);
  require(burst.source.cursor == 2 && burst.runtime->completed_cycles() == 1 &&
              burst.source.log == std::vector<std::string>{"E630", "E630", "D630"},
          "Finite same-timestamp input burst lost its stable order");
}

void callback_failures() {
  Fixture query;
  query.source.on_query = [&] {
    const auto same = query.dsp.checkpoint();
    query.dsp.restore(same);
  };
  require(rejects([&] { query.step(); }).contains("unchanged native boundary") &&
              query.runtime->faulted() && query.runtime->completed_cycles() == 0,
          "Pure deadline query replaced an indistinguishable held-reset owner");
  Fixture query_reentry;
  query_reentry.source.on_query = [&] { static_cast<void>(query_reentry.runtime->checkpoint()); };
  require(rejects([&] { query_reentry.step(); }).contains("healthy inactive") &&
              query_reentry.runtime->faulted(),
          "Deadline query reentered an active native checkpoint");
  Fixture reentry(DspPeerOrder::peer_first, {first_cycle - 1});
  reentry.source.on_observe = [&](std::uint64_t) {
    static_cast<void>(reentry.runtime->checkpoint());
  };
  reentry.step();
  require(rejects([&] { reentry.step(); }).contains("healthy inactive") &&
              reentry.source.cursor == 0 && reentry.runtime->faulted(),
          "External observation reentered an active native coordinator");
  Fixture mutated;
  const auto chip = mutated.dsp.checkpoint();
  mutated.provide_inputs = [&] {
    mutated.dsp.restore(chip);
    return cycle_inputs();
  };
  mutated.step();
  require(rejects([&] { mutated.step(); }).contains("unchanged native boundary") &&
              mutated.runtime->completed_cycles() == 0 && mutated.runtime->faulted(),
          "Cycle provider replaced native ownership without a visible failure");
  Fixture observation(DspPeerOrder::peer_first, {first_cycle - 1});
  observation.step();
  observation.source.on_observe = [&](std::uint64_t) {
    const auto same = observation.dsp.checkpoint();
    observation.dsp.restore(same);
  };
  require(rejects([&] { observation.step(); }).contains("unchanged native boundary") &&
              observation.runtime->faulted() && observation.source.cursor == 1,
          "Same-state chip restoration hid a lost held-reset timeline owner");
  Fixture observer;
  observer.on_cycle = [&](const DspRuntimeCycle &) {
    const auto same = observer.dsp.checkpoint();
    observer.dsp.restore(same);
  };
  observer.step();
  require(rejects([&] { observer.step(); }).contains("unchanged native boundary") &&
              observer.runtime->faulted() && observer.runtime->completed_cycles() == 1,
          "Native completion observer replaced ownership after a genuine committed DSP cycle");
}
}

void guards() {
  malformed_sources();
  retained_source();
  pinned_and_monotone();
  causal_append();
  budgets();
  callback_failures();
}
}
