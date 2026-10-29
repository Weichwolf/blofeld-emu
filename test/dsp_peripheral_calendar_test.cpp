// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_timing.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr std::uint64_t event = 10, second = 20, next_event = 25, third = 30, postponed = 35,
                        later = 40;
constexpr unsigned gigahertz = 1000000000;
constexpr std::uint64_t selected = 20, fall = 100, valid = 130, rise = 140, hold = 155, hiz = 170,
                        finish_time = 180;
constexpr NandByte own{.value = 0xa3, .known = 0xff};

void require(const bool value, const std::string_view reason) {
  if (!value) { throw Fault(std::string(reason)); }
}

void rejects(PeripheralCalendar &calendar, const auto &operation) {
  const auto before = calendar.state();
  bool rejected = false;
  try {
    operation();
  } catch (const Fault &) { rejected = true; }
  require(rejected && calendar.state() == before,
          "Directed refusal preserves every calendar value");
}

void literal_history() {
  const auto id = std::make_shared<const PeripheralEventIdentity>();
  PeripheralCalendar calendar;
  calendar.bind(id, event);
  require(!calendar.first(event, event, {.ticks = event}),
          "Inherited input wins a three-way exact tie");
  calendar.caused(PeripheralCalendarCause::external, event, event);
  require(calendar.first(event, std::nullopt, {.ticks = event}),
          "Internal input precedes same-time native work");
  calendar.caused(PeripheralCalendarCause::peripheral, event, std::nullopt);
  calendar.caused(PeripheralCalendarCause::native, event, next_event);
  calendar.finish({.ticks = event});
  const PeripheralCalendarState expected{.horizon = {.ticks = event},
                                         .identity = id,
                                         .pending = next_event,
                                         .last_native = event,
                                         .last_external = event,
                                         .last_peripheral = event,
                                         .causes = 3,
                                         .peripheral_events = 1,
                                         .advanced = true};
  require(calendar.state() == expected, "All literal three-actor endpoints and counters match");
  const auto snapshot = calendar.checkpoint();
  rejects(calendar, [&]() { calendar.verify(id, postponed); });
  rejects(calendar, [&]() { calendar.finish({.ticks = third}); });
  rejects(calendar, [&]() { calendar.caused(PeripheralCalendarCause::native, next_event, later); });
  rejects(calendar, [&]() { calendar.caused(PeripheralCalendarCause::native, second, second); });
  calendar.caused(PeripheralCalendarCause::native, second, postponed);
  calendar.finish({.ticks = second});
  calendar.caused(PeripheralCalendarCause::native, third, postponed);
  calendar.finish({.ticks = third});
  calendar.caused(PeripheralCalendarCause::peripheral, postponed, later);
  calendar.finish({.ticks = postponed, .native_through = false, .events_through = true});
  require(calendar.state().causes == 6 && calendar.state().peripheral_events == 2 &&
              calendar.state().pending == later && calendar.state().last_native == third,
          "Native cause reschedules a future deadline,without a fabricated peripheral event");
  rejects(calendar, [&]() { calendar.restore(snapshot, id, next_event, {.ticks = second}); });
  const auto other = std::make_shared<const PeripheralEventIdentity>();
  rejects(calendar, [&]() { calendar.restore(snapshot, other, next_event, {.ticks = event}); });
  calendar.restore(snapshot, id, next_event, {.ticks = event});
  require(calendar.state() == expected, "Restore retains complete known independent value state");
  PeripheralCalendar clone(snapshot, id, next_event, {.ticks = event});
  require(clone.state() == expected, "Trusted distinct clone retains every literal endpoint");
  rejects(clone, [&]() { clone.restore(snapshot, id, next_event, {.ticks = event}); });
  const auto clone_snapshot = clone.checkpoint();
  clone.caused(PeripheralCalendarCause::native, second, postponed);
  clone.finish({.ticks = second});
  clone.restore(clone_snapshot, id, next_event, {.ticks = event});
  require(clone.state() == expected && calendar.state() == expected,
          "Clone restore leaves source owner unchanged");
}

void boundary_guards() {
  const auto id = std::make_shared<const PeripheralEventIdentity>();
  PeripheralCalendar calendar;
  rejects(calendar, [&]() { calendar.bind({}, event); });
  rejects(calendar, [&]() { static_cast<void>(calendar.checkpoint()); });
  calendar.bind(id, 0);
  const auto initial = calendar.checkpoint();
  rejects(calendar, [&]() { calendar.finish({.ticks = 0}); });
  calendar.finish({.ticks = 0, .native_through = false, .events_through = false});
  require(calendar.first(
              0, std::nullopt, {.ticks = 0, .native_through = false, .events_through = true}),
          "Split input boundary remains available");
  rejects(calendar, [&]() {
    static_cast<void>(calendar.first(
        0, std::nullopt, {.ticks = 0, .native_through = true, .events_through = false}));
  });
  calendar.caused(PeripheralCalendarCause::peripheral, 0, std::nullopt);
  calendar.finish({.ticks = 0, .native_through = false, .events_through = true});
  rejects(calendar, [&]() { calendar.caused(PeripheralCalendarCause::external, 0, std::nullopt); });
  calendar.caused(PeripheralCalendarCause::native, 0, event);
  calendar.finish({.ticks = 0});
  rejects(calendar, [&]() {
    calendar.finish({.ticks = 0, .native_through = false, .events_through = true});
  });
  calendar.restore(initial, id, 0, {});
  require(calendar.state().causes == 0 && !calendar.state().advanced &&
              calendar.state().pending == 0,
          "Initial before-work snapshot retains an unconsumed event at zero");
}

void suffix(NandReadTiming &model,
            PeripheralCalendar &calendar,
            const std::shared_ptr<const PeripheralEventIdentity> &id) {
  require(model.output(hold) == NandReadOutput{}, "Literal minimum-hold end loses drive knowledge");
  calendar.caused(PeripheralCalendarCause::peripheral, hold, model.next_transition());
  calendar.finish({.ticks = hold, .native_through = false, .events_through = true});
  require(model.output(hiz) == NandReadOutput{.driven = false},
          "Literal release maximum establishes Hi-Z");
  calendar.caused(PeripheralCalendarCause::peripheral, hiz, model.next_transition());
  calendar.finish({.ticks = hiz, .native_through = false, .events_through = true});
  calendar.caused(PeripheralCalendarCause::native, finish_time, model.next_transition());
  calendar.finish({.ticks = finish_time});
  calendar.verify(id, std::nullopt);
  require(calendar.state().causes == 8 && calendar.state().peripheral_events == 4 &&
              calendar.state().last_native == finish_time &&
              calendar.state().last_peripheral == hiz,
          "Replay ends at independent literal cause/event counts and native/peripheral times");
}

void peer_causes() {
  const auto id = std::make_shared<const PeripheralEventIdentity>();
  PeripheralCalendar calendar;
  calendar.bind(id, std::nullopt);
  calendar.caused(PeripheralCalendarCause::native, event, std::nullopt);
  calendar.finish({.ticks = event});
  rejects(calendar, [&]() { calendar.caused(PeripheralCalendarCause::peer, event, event); });
  calendar.caused(PeripheralCalendarCause::peer, event, next_event);
  calendar.finish({.ticks = event});
  const PeripheralCalendarState expected{.horizon = {.ticks = event},
                                         .identity = id,
                                         .pending = next_event,
                                         .last_native = event,
                                         .last_peer = event,
                                         .causes = 2,
                                         .advanced = true};
  require(calendar.state() == expected, "A real peer cause may follow same-tick native completion");
  const auto snapshot = calendar.checkpoint();
  calendar.caused(PeripheralCalendarCause::peer, second, postponed);
  calendar.finish({.ticks = second});
  calendar.restore(snapshot, id, next_event, {.ticks = event});
  require(calendar.state() == expected, "Owned checkpoint includes the full peer-write history");
  rejects(calendar, [&]() { calendar.caused(PeripheralCalendarCause::peer, event, later); });
  PeripheralCalendar before;
  before.bind(id, second);
  before.finish({.ticks = second, .native_through = false, .events_through = false});
  before.caused(PeripheralCalendarCause::peer, second, second);
  before.caused(PeripheralCalendarCause::external, second, second);
  before.caused(PeripheralCalendarCause::peripheral, second, std::nullopt);
  before.caused(PeripheralCalendarCause::native, second, std::nullopt);
  before.finish({.ticks = second});
  const auto state = before.state();
  require(state.causes == 4 && state.peripheral_events == 1 && state.last_peer == second &&
              state.last_external == second && state.last_peripheral == second &&
              state.last_native == second,
          "Peer-before-input preserves an already offered exact input and full four-actor state");
}

void actual_NAND_state() {
  const auto id = std::make_shared<const PeripheralEventIdentity>();
  NandReadTiming model(gigahertz);
  PeripheralCalendar calendar;
  model.observe(0, {.ce_n = true, .re_n = true});
  calendar.bind(id, model.next_transition());
  require(model.output(selected) == NandReadOutput{.driven = false},
          "Actual initial CE release bound");
  calendar.caused(PeripheralCalendarCause::peripheral, selected, model.next_transition());
  calendar.finish({.ticks = selected, .native_through = false, .events_through = true});
  model.observe(selected, {.ce_n = false, .re_n = true});
  calendar.caused(PeripheralCalendarCause::native, selected, model.next_transition());
  calendar.finish({.ticks = selected});
  unsigned calls = 0;
  model.observe(fall, {.ce_n = false, .re_n = false}, [&]() {
    ++calls;
    return own;
  });
  calendar.caused(PeripheralCalendarCause::native, fall, model.next_transition());
  calendar.finish({.ticks = fall});
  require(model.output(valid) == NandReadOutput{.driven = true, .data = own},
          "Actual own byte becomes valid");
  calendar.caused(PeripheralCalendarCause::peripheral, valid, model.next_transition());
  calendar.finish({.ticks = valid, .native_through = false, .events_through = true});
  model.observe(rise, {.ce_n = false, .re_n = true});
  calendar.caused(PeripheralCalendarCause::native, rise, model.next_transition());
  calendar.finish({.ticks = rise});
  require(calendar.state().causes == 5 && calendar.state().peripheral_events == 2 &&
              calendar.state().pending == hold && calls == 1,
          "Independently known pending hold snapshot");
  const auto board = model;
  const auto checkpoint = calendar.checkpoint();
  suffix(model, calendar, id);
  const auto final_model = model;
  const auto final_calendar = calendar.state();
  for (unsigned replay = 0; replay < 2; ++replay) {
    model = board;
    calendar.restore(checkpoint, id, model.next_transition(), {.ticks = rise});
    suffix(model, calendar, id);
    require(
        model == final_model && calendar.state() == final_calendar && calls == 1,
        "Joint actual NAND/calendar restoration repeats exact state without another source read");
  }
}
}

int main() {
  try {
    literal_history();
    boundary_guards();
    peer_causes();
    actual_NAND_state();
    std::cout << "PASS owned generic peripheral calendar:independent literal actor order,"
                 "causality/refusal guards,complete owned snapshots,distinct clone,"
                 "two actual NAND hold/Hi-Z joint replays;no original boot claim\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL owned calendar: " << error.what() << '\n';
    return 1;
  }
}
