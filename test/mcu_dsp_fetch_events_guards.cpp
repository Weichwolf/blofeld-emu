// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
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
constexpr auto dm = McuWriteOrder::before_mcu;
constexpr auto md = McuWriteOrder::after_mcu;
constexpr auto em = McuWriteOrder::before_mcu;
constexpr auto me = McuWriteOrder::after_mcu;
constexpr auto ed = DspPeerOrder::peer_first;
constexpr auto de = DspPeerOrder::dsp_first;
constexpr Case baseline{
    .native = dm, .external = em, .peers = ed, .sample = 2, .order = {'E', 'D', 'M'}};

void collision_failures() {
  constexpr std::array controls{
      Case{.native = dm, .external = me, .peers = ed, .sample = 0, .order = {}},
      Case{.native = md, .external = em, .peers = de, .sample = 0, .order = {}},
      Case{.native = McuWriteOrder::unqualified,
           .external = em,
           .peers = ed,
           .sample = 0,
           .order = {}},
      Case{.native = dm,
           .external = McuWriteOrder::unqualified,
           .peers = ed,
           .sample = 0,
           .order = {}},
      Case{.native = md,
           .external = me,
           .peers = DspPeerOrder::reject_tie,
           .sample = 0,
           .order = {}}};
  for (unsigned index = 0; index < controls.size(); ++index) {
    Fixture f(controls.at(index));
    f.step(1);

    const auto error = rejects([&] { f.step(1); });
    require(error.contains(index == 2 || index == 3 ? "Unqualified simultaneous" : "Cyclic") &&
                f.runtime->faulted() && f.runtime->completed_cycles() == 2 && f.cursor == 0 &&
                f.levels == 0 && f.mcu.bus().read(1) == 0,
            "Unknown/cyclic collision refuses before every tied actor and preserves only genuine "
            "prefix periods");
    static_cast<void>(rejects([&] { f.runtime->end_instruction(); }));
  }
}

void initial_callback_guard(const bool observation) {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  OwnDspMachine dsp{{.extal_hz = oscillator,
                     .timebase_hz = common,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  prepare(mcu, dsp);
  const auto native = reset_registers();
  unsigned calls = 0;
  const auto mutate = [&] {
    ++calls;
    auto state = mcu.cpu().state();
    ++state.pc;
    mcu.cpu().set_state(state);
  };
  DspRuntimeEvents events{.identity = std::make_shared<const bool>(true),
                          .next_ticks = [&]() -> std::optional<std::uint64_t> {
                            if (!observation) { mutate(); }
                            return initial_time;
                          },
                          .observe =
                              [&](std::uint64_t) {
                                if (observation) { mutate(); }
                              },
                          .order = ed,
                          .maximum_events_per_period = 1};
  require(rejects([&] {
            const McuDspRuntime runtime(settings(baseline), mcu, dsp, events);
          }).contains("changed CPU state or time") &&
              calls == 1 && mcu.run_clock().bus_cycles() == 5 && dsp.cpu().registers() == native,
          "Initial event queries and observations cannot mutate either native CPU or time");
}

void past_input() {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  OwnDspMachine dsp{{.extal_hz = oscillator,
                     .timebase_hz = common,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  prepare(mcu, dsp);
  DspRuntimeEvents events{
      .identity = std::make_shared<const bool>(true),
      .next_ticks = []() -> std::optional<std::uint64_t> { return initial_time - 1; },
      .observe = [](std::uint64_t) { throw Fault("A past event must never be observed"); },
      .order = ed,
      .maximum_events_per_period = 1};
  const auto error =
      rejects([&] { const McuDspRuntime runtime(settings(baseline), mcu, dsp, events); });
  require(error.contains("predates its initial native MCU boundary") &&
              error.contains("tick=449 initial-tick=450") && error.contains("mcu-pc=0x8000") &&
              error.contains("dsp-pc=0xff0000") && mcu.run_clock().bus_cycles() == 5 &&
              dsp.clock().native_cycle_units() == 0,
          "A warmed owner refuses past input before observation with both CPU PCs and times");
}

void reentry() {
  Fixture f(baseline);
  unsigned rejected = 0;
  f.after_event = [&] {
    require(
        rejects([&] { f.runtime->end_instruction(); }).contains("healthy inactive native epoch"),
        "External callback cannot reenter the active joint owner");
    ++rejected;
  };
  f.step(4);
  require(rejected == 2 && f.cursor == 2 && !f.runtime->faulted(),
          "Caught recursive requests consume no extra period or input and preserve the owner");
}

void newborn_from_mcu(const McuWriteOrder policy) {
  auto order = baseline;
  order.native = md;
  order.external = policy;
  Fixture f(order, initial_time, {});
  f.mcu.bus().set_observer([&](const auto &event) {
    if (!event.write || event.address != 1) { return; }
    f.runtime->observe_write(
        event,
        [&] {
          f.levels |= 1U;
          f.drive();
          f.ticks = {collision, collision + 1};
        },
        cycle_inputs);
  });
  if (policy == me) {
    f.step(6);
    require(f.cursor == 2 && !f.runtime->faulted() && f.dsp.cpu().registers().x == 3 &&
                f.dsp.cpu().registers().y == 1,
            "A real MCU cause may generate a same-tick event only after that MCU write");
  } else {
    const auto error = rejects([&] { f.step(2); });
    const char *const reason = policy == em ? "moved backwards into a consumed timeline"
                                            : "cannot precede its MCU write cause";
    require(error.contains(reason) && f.runtime->faulted() && f.cursor == 0 &&
                f.runtime->completed_cycles() == 2 && f.mcu.bus().read(1) == 1,
            "A newborn event cannot retroactively precede its actual MCU cause");
  }
}

void newborn_from_dsp() {
  auto order = baseline;
  order.external = McuWriteOrder::unqualified;
  order.peers = de;
  Fixture f(order, initial_time, {});
  f.mcu.set_cycle_observer([&](const auto cycle) {
    f.runtime->begin_cycle(cycle,
                           cycle.kind == 'w' && cycle.address == 1,
                           cycle_inputs,
                           [&](const DspRuntimeCycle &result) {
                             if (result.completion_ticks == collision) {
                               f.ticks.push_back(collision);
                             }
                           });
  });
  require(
      rejects([&] { f.step(2); }).contains("Unqualified newborn external event") &&
          f.runtime->faulted() && f.cursor == 0 && f.runtime->completed_cycles() == 3 &&
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
  newborn_from_mcu(McuWriteOrder::unqualified);
  newborn_from_dsp();
}
} // namespace blofeld::test::joint_events
