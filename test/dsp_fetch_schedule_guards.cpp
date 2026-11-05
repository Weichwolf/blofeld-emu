// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
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
  Fixture fixture(DspPeerOrder::peer_first, {initial_ticks});
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

void advance_outside_runtime(Fixture &fixture) {
  auto request = cycle_inputs();
  const auto last = fixture.dsp.bus().last_dma_cycle();
  request.cycle = last ? *last + 1 : 0;
  static_cast<void>(fixture.dsp.bus().advance_dma(request));
}

void callback_failures() {
  Fixture query;
  query.source.on_query = [&] { advance_outside_runtime(query); };
  require(rejects([&] { query.step(); }).contains("unchanged native boundary") &&
              query.runtime->faulted() && query.runtime->completed_cycles() == 0,
          "Pure deadline query advanced DMA outside the owned native timeline");
  Fixture query_reentry;
  query_reentry.source.on_query = [&] { query_reentry.runtime->end_instruction(); };
  require(rejects([&] { query_reentry.step(); }).contains("healthy inactive") &&
              query_reentry.runtime->faulted(),
          "Deadline query reentered an active native instruction boundary");
  Fixture reentry(DspPeerOrder::peer_first, {first_cycle - 1});
  reentry.source.on_observe = [&](std::uint64_t) { reentry.runtime->end_instruction(); };
  reentry.step();
  require(rejects([&] { reentry.step(); }).contains("healthy inactive") &&
              reentry.source.cursor == 0 && reentry.runtime->faulted(),
          "External observation reentered an active native coordinator");
  Fixture mutated;
  mutated.provide_inputs = [&] {
    advance_outside_runtime(mutated);
    return cycle_inputs();
  };
  mutated.step();
  require(rejects([&] { mutated.step(); }).contains("unchanged native boundary") &&
              mutated.runtime->completed_cycles() == 0 && mutated.runtime->faulted(),
          "Cycle provider advanced DMA outside the owned native boundary");
  Fixture observation(DspPeerOrder::peer_first, {first_cycle - 1});
  observation.step();
  observation.source.on_observe = [&](std::uint64_t) { advance_outside_runtime(observation); };
  require(rejects([&] { observation.step(); }).contains("unchanged native boundary") &&
              observation.runtime->faulted() && observation.source.cursor == 1,
          "External observation changed DMA outside the owned native timeline");
  Fixture observer;
  observer.on_cycle = [&](const DspRuntimeCycle &) { advance_outside_runtime(observer); };
  observer.step();
  require(rejects([&] { observer.step(); }).contains("unchanged native boundary") &&
              observer.runtime->faulted() && observer.runtime->completed_cycles() == 1,
          "Native completion observer changed DMA after a genuine committed DSP cycle");
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
