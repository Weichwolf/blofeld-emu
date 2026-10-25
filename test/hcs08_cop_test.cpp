// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_system_options.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr auto contract = Hcs08CopContract::candidate_gt8a_run_bus_edges;
constexpr unsigned short_periods = 8192, long_periods = 262144, values = 256;
constexpr unsigned short_options = 0x93, long_options = 0xd3, disabled_options = 0x53;
constexpr unsigned pin_source = 0x40, cop_source = 0x20, age = 17;
constexpr ExecutionContext context{.pc = 0xe1c7, .ccr = 0x69};

void periods_and_reset() {
  for (const auto options : {short_options, long_options}) {
    Hcs08Cop cop(contract);
    cop.context(context);
    cop.reset(pin_source);
    cop.write(
        {.address = Hcs08SystemOptions::address, .value = static_cast<std::uint8_t>(options)});
    const auto periods = options == short_options ? short_periods : long_periods;
    require(cop.advance_bus(periods - 1) == periods - 1 && !cop.state().reset_requested &&
                cop.state().elapsed == periods - 1,
            "No reset before the exact2^13/2^18 bus-period deadline");
    const auto before = cop;
    require(cop.advance_bus(std::numeric_limits<std::uint64_t>::max()) == 1 &&
                cop.state().reset_requested && cop.state().elapsed == periods,
            "A batched event stops at the first reset request and returns its consumed prefix");
    const auto requested = cop.state();
    require(
        rejected([&] { cop.write({.address = Hcs08Cop::status}); }).contains("coordinated reset") &&
            rejected([&] {
              static_cast<void>(cop.advance_bus(1));
            }).contains("coordinated reset") &&
            rejected([&] {
              static_cast<void>(cop.read(Hcs08Cop::status));
            }).contains("coordinated reset") &&
            cop.state() == requested,
        "A late service/read/clock cannot undo an already requested reset");
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto fresh = before;
      require(fresh.advance_bus(1) == 1 && fresh.state() == requested,
              "Fresh watchdog owners retain exact deadline history");
    }
    cop.reset(cop_source);
    require(cop.state() == Hcs08CopState{.reset_status = cop_source} &&
                cop.read(Hcs08Cop::status) == cop_source,
            "Explicit board reset restores COP/SOPT and supplied reset causes");
  }
}

void service_and_options() {
  require(rejected([] {
            const Hcs08Cop invalid(Hcs08CopContract::unqualified);
          }).contains("run-mode bus-edge contract"),
          "Unknown device/clock contracts cannot activate the watchdog model");
  Hcs08Cop cop(contract);
  cop.context(context);
  require(rejected([&] {
            static_cast<void>(cop.read(Hcs08Cop::status));
          }).contains("unknown reset sources"),
          "Entering a live context cannot invent previous reset flags");
  for (unsigned value = 0; value < values; ++value) {
    require(cop.advance_bus(age) == age, "Known run-mode bus edges age the watchdog");
    cop.write({.address = Hcs08Cop::status, .value = static_cast<std::uint8_t>(value)});
    require(cop.state() == Hcs08CopState{},
            "Every byte services COP without changing unknown SRS or SOPT admission");
  }
  cop.reset(pin_source);
  require(cop.advance_bus(age) == age, "Reference deadline starts at the supplied reset");
  cop.write({.address = Hcs08SystemOptions::address, .value = short_options});
  cop.write({.address = Hcs08SystemOptions::address, .value = 0});
  require(cop.read(Hcs08SystemOptions::address) == short_options && cop.state().options_written &&
              cop.state().elapsed == age,
          "Only the first actual SOPT store changes timeout; later invalid stores have no effect");
  cop.write({.address = Hcs08Cop::status, .value = 0});
  require(cop.advance_bus(short_periods) == short_periods && cop.state().reset_requested,
          "Servicing does not reopen SOPT or silently restore the long timeout");
  cop.reset();
  require(cop.advance_bus(short_periods + 1) == short_periods + 1,
          "Long timeout still runs before SOPT's first write");
  const auto before = cop.state();
  require(rejected([&] {
            cop.write({.address = Hcs08SystemOptions::address, .value = short_options});
          }).contains("counter history") &&
              cop.state() == before,
          "Shortening across an already elapsed threshold requires unmodeled divider history");
  cop.write({.address = Hcs08SystemOptions::address, .value = disabled_options});
  require(cop.advance_bus(std::numeric_limits<std::uint64_t>::max()) ==
                  std::numeric_limits<std::uint64_t>::max() &&
              !cop.state().reset_requested,
          "A real first COPE=0 store disables reset requests");
  cop.reset(pin_source);
  const auto reset = cop.state();
  require(rejected([&] { cop.reset(1); }).contains("bit0") && cop.state() == reset,
          "Impossible reserved reset bit rejects before state mutation");
  const auto error = rejected([&] { cop.write({.address = Hcs08Cop::status + 1}); });
  require(error.contains("write address=0x1801 pc=0xe1c7 ccr=0x0069") && cop.state() == reset,
          "Unsupported registers report operation/address/PC/context without side effects");
}
} // namespace

int main() {
  try {
    periods_and_reset();
    service_and_options();
    std::cout << "COP service, deadline, SOPT and explicit reset-source histories verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
