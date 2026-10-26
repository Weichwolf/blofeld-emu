// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_icg_history.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned status1 = 0x4a, status2 = 0x4b, armed = 4, source_mask = 3;
constexpr unsigned context_pc = 0xe1b0, context_ccr = 0x69, flag_bits = 0x15;
constexpr unsigned observed_status = 0x6a, reserved_status2 = 0xfe, reset_routes = 0x88;
constexpr std::array routes{0U, 0x80U, 8U, 0x88U};
constexpr std::array faults{Hcs08IcgFault::unqualified,
                            Hcs08IcgFault::loss_of_lock,
                            Hcs08IcgFault::loss_of_clock,
                            Hcs08IcgFault::loss_of_clock_and_lock};

Hcs08IcgHistory owner() {
  Hcs08IcgHistory result(Hcs08IcgHistoryContract::candidate_gt8a_unknown_events);
  result.context({.pc = context_pc, .ccr = context_ccr});
  return result;
}

struct Reference {
  std::bitset<8> states{1};
  unsigned routing = 0;

  void event(const unsigned sources) {
    std::bitset<8> result;
    for (unsigned index = 0; index < states.size(); ++index) {
      if (!states[index]) { continue; }
      const auto latch = (sources & ~routing) != 0 ? 0U : index & armed;
      result.set((index & source_mask) | sources | latch);
    }
    states = result;
  }

  void interval() {
    const auto previous = *this;
    for (unsigned sources = 1; sources <= source_mask; ++sources) {
      auto branch = previous;
      branch.event(sources);
      states |= branch.states;
    }
  }

  void read() {
    std::bitset<8> result;
    for (unsigned index = 0; index < states.size(); ++index) {
      if (!states[index]) { continue; }
      const auto sources = index & source_mask;
      result.set(sources | ((sources & ~routing) != 0 ? armed : 0U));
    }
    states = result;
  }

  void clear() {
    std::bitset<8> result;
    for (unsigned index = 0; index < states.size(); ++index) {
      if (!states[index]) { continue; }
      const auto next = (index & armed) != 0 ? index & routing : index;
      result.set(next);
    }
    states = result;
  }

  [[nodiscard]] unsigned flags(const unsigned index) const {
    const auto sources = index & source_mask;
    return ((sources & 1U) != 0 ? 0x10U : 0U) | ((sources & 2U) != 0 ? 4U : 0U) |
           ((sources & ~routing) != 0 ? 1U : 0U);
  }

  [[nodiscard]] Hcs08KnownByte byte() const {
    unsigned value = flag_bits;
    unsigned any = 0;
    for (unsigned index = 0; index < states.size(); ++index) {
      if (!states[index]) { continue; }
      value &= flags(index);
      any |= flags(index);
    }
    return {.value = static_cast<std::uint8_t>(value),
            .known = static_cast<std::uint8_t>(flag_bits & ~(value ^ any))};
  }

  [[nodiscard]] unsigned actions(const bool masked) const {
    unsigned result = 0;
    for (unsigned index = 0; index < states.size(); ++index) {
      if (!states[index]) { continue; }
      const auto sources = index & source_mask;
      unsigned action = 1;
      if ((sources & routing) != 0) {
        action = 4;
      } else if (!masked && (sources & ~routing) != 0) {
        action = 2;
      }
      result |= action;
    }
    return result;
  }
};

void check(const Hcs08IcgHistory &history, const Reference &reference) {
  auto copy = history;
  const auto byte = copy.read_known(status1);
  const auto expected = reference.byte();
  require(byte.value == expected.value && byte.known == expected.known,
          "Every reported event bit agrees with all independent concrete latch histories");
  for (const bool masked : {false, true}) {
    const auto actions = reference.actions(masked);
    if (actions != 1 && actions != 2 && actions != 4) {
      require(rejected([&] {
                static_cast<void>(history.action(masked));
              }).contains("unobserved event history affects IRQ/reset selection"),
              "An ambiguous interrupt or reset must fail visibly");
      continue;
    }
    const auto actual = history.action(masked);
    require((actions == 1 && actual == Hcs08IcgFaultAction::none) ||
                (actions == 2 && actual == Hcs08IcgFaultAction::interrupt) ||
                (actions == 4 && actual == Hcs08IcgFaultAction::reset),
            "Known effective actions preserve real reset priority and CPU interrupt masking");
  }
}

void operation(Hcs08IcgHistory &history, Reference &reference, const unsigned choice) {
  switch (choice) {
    case 0:
      history.unobserved_interval();
      reference.interval();
      return;
    case 1:
      static_cast<void>(history.read_known(status1));
      reference.read();
      return;
    case 2:
      history.write({.address = status1, .value = 1});
      reference.clear();
      return;
    case 3: static_cast<void>(history.read_known(status2)); return;
    case 4:
    case 5:
    case 6:
      history.signal(faults.at(choice - 3));
      reference.event(choice - 3);
      return;
    case 7: history.write({.address = status1, .value = 0}); return;
    default: throw Fault("Invalid independent operation grid");
  }
}

void grid() {
  constexpr unsigned sequence_count = 32768;
  constexpr unsigned steps = 5;
  constexpr unsigned radix = 8;
  for (unsigned routing = 0; routing < routes.size(); ++routing) {
    for (unsigned sequence = 0; sequence < sequence_count; ++sequence) {
      auto history = owner();
      history.control2(static_cast<std::uint8_t>(routes[routing]));
      Reference reference{.routing = routing};
      auto remaining = sequence;
      for (unsigned step = 0; step < steps; ++step) {
        operation(history, reference, remaining % radix);
        remaining /= radix;
        check(history, reference);
      }
    }
  }
}

void explicit_sequences() {
  auto history = owner();
  history.observe({.mode = 1,
                   .reference_crystal = true,
                   .locked = true,
                   .external_stable = true,
                   .dco_stable = true});
  require(history.read(status1) == observed_status && history.read(status2) == 1,
          "A fresh fully observed point is readable without fabricating a running clock");
  history.unobserved_interval();
  auto byte = history.read_known(status1);
  require(byte.known == 0 && byte.value == 0,
          "An unobserved interval discards old mode/reference/lock and all fault assumptions");
  byte = history.read_known(status2);
  require(byte.known == reserved_status2 && byte.value == 0,
          "Only documented reserved ICGS2 bits remain known after unobserved time");
  const auto snapshot = history;
  history.write({.address = status1, .value = 1});
  require(history.read_known(status1).known == flag_bits,
          "A completed real read/clear sequence proves interrupt-routed latches clear");
  auto restored = snapshot;
  restored.unobserved_interval();
  restored.write({.address = status1, .value = 1});
  require(restored.read_known(status1).known == 0,
          "A new possible interrupt invalidates an old read even in a fresh copied owner");
}

void refused_sequences() {
  auto history = owner();
  history.unobserved_interval();
  const auto failure = rejected([&] { static_cast<void>(history.read(status1)); });
  require(failure.contains("address=0x004a pc=0xe1b0 ccr=0x0069 agent=CPU") &&
              failure.contains("unknown live bits known=0x0000"),
          "Strict unknown reads retain operation, address, actual CPU context and mask");
  history.write({.address = status1, .value = 1});
  require(history.read_known(status1).known == 0,
          "Refused strict reads cannot arm an acknowledgement on any possible history");
  require(rejected([&] { history.control2(0x80); }).contains("pending-fault routing change"),
          "An uncertain pending source cannot acquire a guessed reset route");
  require(history.action(true) == Hcs08IcgFaultAction::none,
          "A refused route change cannot partially enable a reset in any possibility");
  require(rejected([&] {
            history.write({.address = status1, .value = 1}, reserved_status2);
          }).contains("unknown ICGIF clear request"),
          "Unknown acknowledgement values refuse before modifying latch histories");
  history.reset();
  require(history.read_known(status1).known == flag_bits &&
              history.action(false) == Hcs08IcgFaultAction::none,
          "Reset restores fault latches and routes but no continuous clock observations");
  history.control2(reset_routes);
  history.unobserved_interval();
  require(rejected([&] { static_cast<void>(history.action(true)); }).contains("IRQ/reset"),
          "CPU masking cannot suppress a possible reset");
}
} // namespace

int main() {
  try {
    grid();
    explicit_sequences();
    refused_sequences();
    std::cout << "Verified 131072 five-operation ICG histories and strict unknown actions.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
