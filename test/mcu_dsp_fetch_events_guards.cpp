// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_dsp_fetch_events_fixture.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace blofeld::test::joint_events {
namespace {
constexpr auto dm = McuDspTieOrder::dsp_before_mcu;
constexpr auto md = McuDspTieOrder::mcu_before_dsp;
constexpr auto em = McuExternalWriteOrder::external_before_mcu;
constexpr auto me = McuExternalWriteOrder::mcu_before_external;
constexpr auto ed = DspExternalOrder::external_before_dsp;
constexpr auto de = DspExternalOrder::dsp_before_external;
constexpr Case baseline{
    .native = dm, .external = em, .peers = ed, .sample = 2, .order = {'E', 'D', 'M'}};

void collision_failures() {
  constexpr std::array controls{
      Case{.native = dm, .external = me, .peers = ed, .sample = 0, .order = {}},
      Case{.native = md, .external = em, .peers = de, .sample = 0, .order = {}},
      Case{.native = McuDspTieOrder::unqualified,
           .external = em,
           .peers = ed,
           .sample = 0,
           .order = {}},
      Case{.native = dm,
           .external = McuExternalWriteOrder::unqualified,
           .peers = ed,
           .sample = 0,
           .order = {}},
      Case{.native = md,
           .external = me,
           .peers = DspExternalOrder::reject_tie,
           .sample = 0,
           .order = {}}};
  for (unsigned index = 0; index < controls.size(); ++index) {
    Fixture f(controls.at(index));
    f.step(1);
    const auto saved = f.schedule.checkpoint();
    std::string first;
    for (unsigned replay = 0; replay < 2; ++replay) {
      f.schedule.restore(saved);
      f.cursor = 0;
      f.levels = 0;
      f.attach();
      const auto error = rejects([&] { f.step(1); });
      require(error.contains(index < 2 ? "Cyclic" : "Unqualified simultaneous") &&
                  f.schedule.faulted() && f.schedule.run().completed_cycles() == 3 &&
                  f.cursor == 0 && f.levels == 0 && f.mcu.bus().read(1) == 0,
              "Unknown/cyclic collision refuses before every tied actor and preserves only genuine "
              "prefix periods");
      static_cast<void>(rejects([&] { f.schedule.end_instruction(); }));
      if (replay == 0) { first = error; }
      require(error == first, "Trusted joint recovery repeats the complete failure frontier");
    }
  }
}

void initial_callback_guard(const bool observation) {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  DspMachine dsp;
  prepare(mcu, dsp);
  const auto native = dsp.cpu().debug_state();
  unsigned calls = 0;
  const auto mutate = [&] {
    ++calls;
    auto state = mcu.cpu().state();
    ++state.pc;
    mcu.cpu().set_state(state);
  };
  DspFetchExternalEvents events{.identity = std::make_shared<const DspFetchEventIdentity>(),
                                .next_ticks = [&]() -> std::optional<std::uint64_t> {
                                  if (!observation) { mutate(); }
                                  return initial_time;
                                },
                                .observe =
                                    [&](std::uint64_t) {
                                      if (observation) { mutate(); }
                                    },
                                .order = ed,
                                .maximum_events = 1};
  require(rejects([&] {
            const McuDspFetchSchedule schedule(settings(baseline, epoch), mcu, dsp, events);
          }).contains("changed native MCU") &&
              calls == 1 && mcu.run_clock().bus_cycles() == 5 && dsp.cpu().debug_state() == native,
          "Initial event queries and observations cannot mutate either native CPU or time");
}

void past_input() {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  DspMachine dsp;
  prepare(mcu, dsp);
  DspFetchExternalEvents events{
      .identity = std::make_shared<const DspFetchEventIdentity>(),
      .next_ticks = []() -> std::optional<std::uint64_t> { return initial_time - 1; },
      .observe = [](std::uint64_t) { throw Fault("A past event must never be observed"); },
      .order = ed,
      .maximum_events = 1};
  require(rejects([&] {
            const McuDspFetchSchedule schedule(settings(baseline, epoch), mcu, dsp, events);
          }).contains("predates its initial native MCU boundary"),
          "A warmed owner cannot replay past input into already executed MCU reset time");
}

void reentry() {
  Fixture f(baseline);
  unsigned rejected = 0;
  f.after_event = [&] {
    require(rejects([&] { f.schedule.end_instruction(); }).contains("healthy inactive boundary"),
            "External callback cannot reenter the active joint owner");
    ++rejected;
  };
  f.step(4);
  require(rejected == 2 && f.cursor == 2 && !f.schedule.faulted(),
          "Caught recursive requests consume no extra period or input and preserve the owner");
}

void newborn_from_mcu(const McuExternalWriteOrder policy) {
  auto order = baseline;
  order.native = md;
  order.external = policy;
  Fixture f(order, epoch, {});
  f.mcu.bus().set_observer([&](const auto &event) {
    if (!event.write || event.address != 1) { return; }
    f.schedule.observe_write(
        event,
        [&] {
          f.levels |= 1U;
          f.drive();
          f.ticks = {collision, collision + 1};
        },
        [] { return fetch_clock::input(); });
  });
  if (policy == me) {
    f.step(4);
    require(f.cursor == 2 && !f.schedule.faulted() && (f.dsp.cpu().state().x & word_mask) == 3 &&
                (f.dsp.cpu().state().y & word_mask) == 1,
            "A real MCU cause may generate a same-tick event only after that MCU write");
  } else {
    require(rejects([&] { f.step(2); }).contains("cannot precede its completed MCU") &&
                f.schedule.faulted() && f.cursor == 0 && f.schedule.run().completed_cycles() == 3 &&
                f.mcu.bus().read(1) == 1,
            "A newborn event cannot retroactively precede its actual MCU cause");
  }
}

void newborn_from_dsp() {
  auto order = baseline;
  order.external = McuExternalWriteOrder::unqualified;
  order.peers = de;
  Fixture f(order, epoch, {});
  f.mcu.set_cycle_observer([&](const auto cycle) {
    f.schedule.begin_cycle(
        cycle,
        cycle.kind == 'w' && cycle.address == 1,
        [] { return fetch_clock::input(); },
        [&](const DspFetchCycleResult &result) {
          if (epoch + result.completion_ticks == collision) { f.ticks.push_back(collision); }
        });
  });
  require(
      rejects([&] { f.step(2); }).contains("Unqualified newborn external event") &&
          f.schedule.faulted() && f.cursor == 0 && f.schedule.run().completed_cycles() == 4 &&
          f.mcu.bus().read(1) == 0,
      "A new event caused by DSP completion still needs ordering before an unexecuted MCU write");
}
}

void guard_cases() {
  collision_failures();
  initial_callback_guard(false);
  initial_callback_guard(true);
  past_input();
  reentry();
  newborn_from_mcu(me);
  newborn_from_mcu(em);
  newborn_from_mcu(McuExternalWriteOrder::unqualified);
  newborn_from_dsp();
}
} // namespace blofeld::test::joint_events
