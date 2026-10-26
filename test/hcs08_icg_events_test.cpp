// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_icg_events.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using Action = Hcs08IcgFaultAction;
using Event = Hcs08IcgFault;
constexpr auto contract = Hcs08IcgEventContract::candidate_gt8a_fault_latches;
constexpr unsigned byte_mask = 0xff, fault_flag_mask = 0x15;
constexpr unsigned lock_route_bit = 0x80, clock_route_bit = 8;
constexpr unsigned pc = 0x9000, ccr = 0x68;
unsigned cases = 0, suffixes = 0, refusals = 0;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

template <class Callback> std::string rejected(Callback callback) {
  try {
    callback();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected explicit unsupported ICG fault-latch operation");
}

struct Expected {
  unsigned pending, acknowledged;
  Action before, after;
};

// ICGS1 LOLS/LOCS/ICGIF: literal source/routing truth table from GT9.3.3/9.4.8/.9.
constexpr std::array events{
    Event::loss_of_lock, Event::loss_of_clock, Event::loss_of_clock_and_lock};
constexpr std::array<std::array<Expected, 3>, 4> matrix{{
    {{{.pending = 0x11, .acknowledged = 0, .before = Action::interrupt, .after = Action::none},
      {.pending = 0x05, .acknowledged = 0, .before = Action::interrupt, .after = Action::none},
      {.pending = 0x15, .acknowledged = 0, .before = Action::interrupt, .after = Action::none}}},
    {{{.pending = 0x10, .acknowledged = 0x10, .before = Action::reset, .after = Action::reset},
      {.pending = 0x05, .acknowledged = 0, .before = Action::interrupt, .after = Action::none},
      {.pending = 0x15, .acknowledged = 0x10, .before = Action::reset, .after = Action::reset}}},
    {{{.pending = 0x11, .acknowledged = 0, .before = Action::interrupt, .after = Action::none},
      {.pending = 0x04, .acknowledged = 0x04, .before = Action::reset, .after = Action::reset},
      {.pending = 0x15, .acknowledged = 0x04, .before = Action::reset, .after = Action::reset}}},
    {{{.pending = 0x10, .acknowledged = 0x10, .before = Action::reset, .after = Action::reset},
      {.pending = 0x04, .acknowledged = 0x04, .before = Action::reset, .after = Action::reset},
      {.pending = 0x14, .acknowledged = 0x14, .before = Action::reset, .after = Action::reset}}},
}};

unsigned routing(const unsigned control) {
  return ((control & lock_route_bit) != 0 ? 1U : 0U) | ((control & clock_route_bit) != 0 ? 2U : 0U);
}

void check_flags(Hcs08IcgEvents &unit, const unsigned expected) {
  const auto actual = unit.read_status_flags();
  require(actual.known == fault_flag_mask && actual.value == expected &&
              (actual.value & ~actual.known) == 0,
          "Only documented fault bits are supplied; CLKST/REFST/LOCK/ERCS stay outside this owner");
}

struct Input {
  unsigned control, event, write;
  bool read;
};

void one(const Input input) {
  const auto [control, event, write, read] = input;
  Hcs08IcgEvents unit(contract);
  unit.control2(static_cast<std::uint8_t>(control));
  unit.signal(events[event]);
  const auto expected = matrix[routing(control)][event];
  require(unit.action() == expected.before, "Reset takes priority over a simultaneous IRQ");
  if (read) { check_flags(unit, expected.pending); }
  const auto before = unit.state();
  unit.write_status(static_cast<std::uint8_t>(write));
  const bool clear = read && (write & 1U) != 0;
  require(unit.action() == (clear ? expected.after : expected.before),
          "Only an armed one-write can acknowledge interrupt-routed faults");
  if (!clear) { require(unit.state() == before, "All other status bits and ICGIF=0 are inert"); }
  check_flags(unit, clear ? expected.acknowledged : expected.pending);
  ++cases;
}

void byte_matrix() {
  for (unsigned control = 0; control <= byte_mask; ++control) {
    for (unsigned event = 0; event < events.size(); ++event) {
      for (unsigned write = 0; write <= byte_mask; ++write) {
        one({.control = control, .event = event, .write = write, .read = false});
        one({.control = control, .event = event, .write = write, .read = true});
      }
    }
  }
}

void renewed_interrupts() {
  for (const auto first : events) {
    for (const auto second : events) {
      Hcs08IcgEvents unit(contract);
      unit.signal(first);
      static_cast<void>(unit.read_status_flags());
      require(unit.state().interrupt_read_armed, "Real pending interrupt arms on status read");
      unit.signal(second);
      require(!unit.state().interrupt_read_armed,
              "A new IRQ, including the same still-latched source, invalidates the old read");
      const auto saved = unit.state();
      unit.write_status(1);
      require(unit.state() == saved && unit.action() == Action::interrupt,
              "A stale read cannot clear either the original or newly signaled cause");
      static_cast<void>(unit.read_status_flags());
      unit.write_status(1);
      require(unit.state() == Hcs08IcgEventState{} && unit.action() == Action::none,
              "A fresh status read followed by one acknowledges the combined interrupt history");
      ++cases;
    }
  }
}

void reset_during_clear() {
  Hcs08IcgEvents unit(contract);
  unit.control2(lock_route_bit);
  unit.signal(Event::loss_of_clock);
  check_flags(unit, 5);
  unit.signal(Event::loss_of_lock);
  require(unit.state().interrupt_read_armed && unit.action() == Action::reset,
          "A reset-routed event retains the prior interrupt read and requests actual board reset");
  unit.write_status(1);
  check_flags(unit, 0x10);
  require(unit.action() == Action::reset, "Software IRQ acknowledgement cannot release reset");
  unit.reset();
  require(unit.state() == Hcs08IcgEventState{} && unit.action() == Action::none,
          "Explicit MCU reset clears fault flags, acknowledgement and control routing together");
  ++cases;
}

void snapshots() {
  for (unsigned control = 0; control <= byte_mask; ++control) {
    for (unsigned event = 0; event < events.size(); ++event) {
      for (const bool arm : {false, true}) {
        Hcs08IcgEvents original(contract);
        original.control2(static_cast<std::uint8_t>(control));
        original.signal(events[event]);
        if (arm) { static_cast<void>(original.read_status_flags()); }
        auto fresh = original;
        require(fresh.state() == original.state(),
                "Fresh copy owns the complete latch/read history");
        original.write_status(1);
        fresh.write_status(1);
        const auto expected = matrix[routing(control)][event];
        require(fresh.state() == original.state() && fresh.action() == original.action(),
                "Trusted joint checkpoint suffix never loses or repeats an acknowledgement");
        check_flags(fresh, arm ? expected.acknowledged : expected.pending);
        check_flags(original, arm ? expected.acknowledged : expected.pending);
        require(fresh.state() == original.state(),
                "Both actual reads arm their own copied histories");
        original.reset();
        require(fresh.state() != original.state() || fresh.action() == Action::none,
                "A reset of one owner cannot mutate the copied owner");
        ++suffixes;
      }
    }
  }
}

void unlatched_routes() {
  for (unsigned source = 0; source < 2; ++source) {
    Hcs08IcgEvents unit(contract);
    unit.signal(events[source]);
    check_flags(unit, matrix[0][source].pending);
    const auto before = unit.state();
    unit.control2(static_cast<std::uint8_t>(source == 0 ? clock_route_bit : lock_route_bit));
    require(unit.state().sources == before.sources && unit.state().interrupt_read_armed &&
                unit.action() == Action::interrupt,
            "Unlatched source routing cannot alter a pending source or its valid read history");
    unit.write_status(1);
    check_flags(unit, 0);
    require(unit.action() == Action::none, "Previously armed source still acknowledges once");
    ++cases;
  }
}

void unsupported() {
  static_cast<void>(
      rejected([] { const Hcs08IcgEvents unit(Hcs08IcgEventContract::unqualified); }));
  ++refusals;
  Hcs08IcgEvents unit(contract);
  unit.context({.pc = pc, .ccr = ccr});
  unit.signal(Event::loss_of_lock);
  static_cast<void>(unit.read_status_flags());
  const auto before = unit.state();
  for (const auto event : {Event::unqualified, static_cast<Event>(byte_mask)}) {
    const auto error = rejected([&] { unit.signal(event); });
    require(error.contains("event address=0x004a") && error.contains("pc=0x9000") &&
                error.contains("ccr=0x0068") && error.contains("agent=CPU") &&
                unit.state() == before,
            "Invalid detector events refuse with source/context before flag/read-history mutation");
    ++refusals;
  }
  const auto error = rejected([&] { unit.control2(lock_route_bit); });
  require(error.contains("write address=0x0049") && error.contains("pending-fault routing") &&
              unit.state() == before,
          "Unqualified live IRQ/reset-route changes retain the exact pre-transaction state");
  ++refusals;
}
}

int main() {
  try {
    byte_matrix();
    renewed_interrupts();
    reset_during_clear();
    snapshots();
    unlatched_routes();
    unsupported();
    std::cout << "ICG fault latches: " << cases << " fixed cases, " << suffixes
              << " fresh suffixes, " << refusals << " strict controls; no clock/status synthesis\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
