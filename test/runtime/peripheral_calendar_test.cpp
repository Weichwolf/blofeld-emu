// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using Cause = PeripheralCalendarCause;
constexpr std::uint64_t next_deadline = 20;
constexpr std::array causes{Cause::native, Cause::external, Cause::peripheral, Cause::peer};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

template <typename Operation>
void reject(PeripheralCalendar &calendar, const Operation &operation) {
  const auto before = calendar.state();
  bool rejected = false;
  try {
    operation();
  } catch (const Fault &) { rejected = true; }
  require(rejected && calendar.state() == before,
          "A rejected transition must preserve every owned deadline and completed cause");
}

void completed_causes() {
  for (const auto cause : causes) {
    PeripheralCalendar calendar;
    const auto identity = std::make_shared<const PeripheralEventIdentity>();
    calendar.bind(identity, cause == Cause::peripheral ? std::optional{10U} : std::nullopt);
    reject(calendar, [&] { calendar.caused(cause, 10, 9); });
    if (cause != Cause::external) {
      reject(calendar, [&] { calendar.caused(cause, 10, 10); });
    }
    calendar.caused(cause, 10, next_deadline);
    const auto state = calendar.state();
    require(state.pending == next_deadline &&
                state.last_native == (cause == Cause::native ? std::optional{10U} : std::nullopt) &&
                state.last_external ==
                    (cause == Cause::external ? std::optional{10U} : std::nullopt) &&
                state.last_peripheral ==
                    (cause == Cause::peripheral ? std::optional{10U} : std::nullopt) &&
                state.last_peer == (cause == Cause::peer ? std::optional{10U} : std::nullopt),
            "Only the actual completed cause updates its timestamp");
    reject(calendar, [&] { calendar.finish({.ticks = 9}); });
    if (cause != Cause::peer) {
      reject(calendar, [&] {
        calendar.finish({.ticks = 10, .native_through = false, .events_through = false});
      });
    }
    calendar.finish({.ticks = 10});
    calendar.verify(identity, next_deadline);
    reject(calendar, [&] { calendar.finish({.ticks = next_deadline}); });
    reject(calendar, [&] { calendar.caused(Cause::native, next_deadline, {}); });
    calendar.caused(Cause::peripheral, next_deadline, {});
    calendar.finish({.ticks = next_deadline});
    require(!calendar.state().pending && calendar.state().last_peripheral == next_deadline,
            "Only actual dispatch consumes an offered deadline");
  }
}

void same_tick_order() {
  PeripheralCalendar calendar;
  const auto identity = std::make_shared<const PeripheralEventIdentity>();
  calendar.bind(identity, 10);
  calendar.caused(Cause::external, 10, 10);
  calendar.caused(Cause::peer, 10, 10);
  reject(calendar, [&] { calendar.caused(Cause::peer, 10, {}); });
  reject(calendar, [&] { calendar.caused(Cause::native, 10, {}); });
  calendar.caused(Cause::peripheral, 10, {});
  calendar.caused(Cause::native, 10, {});
  reject(calendar, [&] { calendar.caused(Cause::native, 10, {}); });
  reject(calendar, [&] { calendar.caused(Cause::external, 10, {}); });
  calendar.finish({.ticks = 10});
  reject(calendar, [&] { calendar.caused(Cause::peer, 10, 10); });
  reject(calendar, [&] { calendar.finish({.ticks = 10, .native_through = false}); });
  require(calendar.state().last_native == 10 && calendar.state().last_peripheral == 10 &&
              calendar.state().last_external == 10 && calendar.state().last_peer == 10,
          "Inherited input, peer, peripheral and native work retain explicit same-tick order");
}

void open_input_boundary() {
  PeripheralCalendar calendar;
  calendar.bind(std::make_shared<const PeripheralEventIdentity>(), {});
  calendar.finish({.ticks = 10, .native_through = false, .events_through = false});
  calendar.caused(Cause::peer, 10, 10);
  reject(calendar, [&] { calendar.finish({.ticks = 10}); });
  calendar.caused(Cause::peripheral, 10, {});
  calendar.caused(Cause::native, 10, {});
  calendar.finish({.ticks = 10});
  reject(calendar, [&] { calendar.caused(Cause::peer, 10, 10); });
}

void identity_and_initial_deadlines() {
  PeripheralCalendar calendar({.ticks = 10});
  const auto identity = std::make_shared<const PeripheralEventIdentity>();
  reject(calendar, [&] { calendar.bind({}, {}); });
  reject(calendar, [&] { calendar.bind(identity, 9); });
  reject(calendar, [&] { calendar.finish({.ticks = 10}); });
  reject(calendar, [&] { calendar.caused(Cause::native, 11, {}); });
  calendar.bind(identity, 10);
  reject(calendar, [&] { calendar.bind(identity, {}); });
  reject(calendar, [&] { calendar.verify(std::make_shared<const PeripheralEventIdentity>(), 10); });
  calendar.caused(Cause::peripheral, 10, 11);
  calendar.finish({.ticks = 11, .events_through = false});
  calendar.caused(Cause::external, 11, 11);
  calendar.caused(Cause::peripheral, 11, {});
  calendar.finish({.ticks = 11});
}
}

int main() {
  try {
    completed_causes();
    same_tick_order();
    open_input_boundary();
    identity_and_initial_deadlines();
    std::cout << "Peripheral deadline ordering and atomic rejection passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
