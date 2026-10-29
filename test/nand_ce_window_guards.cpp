// SPDX-License-Identifier: GPL-3.0-only
#include "nand_ce_window_fixture.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

#include <cstdint>
#include <iostream>
#include <limits>

namespace blofeld::test::ce_window {
namespace {
constexpr std::uint64_t unexpired_release = initial_rise + 29;
constexpr std::uint64_t expired_release = initial_rise + 30;
constexpr std::uint64_t early_write_hold = initial_rise + 4;
constexpr std::uint64_t expired_write_cycle = initial_fall + 42;
constexpr std::uint64_t early_write_cycle = initial_fall + 41;
constexpr std::uint64_t early_read_setup = 59;

void uninitialized_and_time() {
  NandReadTiming read(ghz);
  NandWriteTiming write(ghz);
  NandReadSetup setup(ghz);
  reject([&] { read.resume_ce_window(0, end, false); }, "initialized");
  reject([&] { write.resume_ce_window(0, end, false); }, "initialized");
  reject([&] { setup.resume_ce_window(0, end, false); }, "initialized");
  auto valid = read_base();
  const auto before = valid;
  reject([&] { valid.resume_ce_window(start, start - 1, false); }, "end precedes start");
  reject([&] { valid.resume_ce_window(deselected - 1, end, false); }, "moved backwards");
  require(valid == before, "Invalid window times preserve every read state field");
  auto output = read_base();
  const auto near_end = std::numeric_limits<std::uint64_t>::max() - 1;
  output.resume_ce_window(start, near_end, false);
  const auto copied = output;
  unsigned calls = 0;
  reject(
      [&] {
        output.observe(near_end + 1, {.ce_n = false, .re_n = false}, [&] {
          ++calls;
          return first;
        });
      },
      "overflow");
  require(output == copied && calls == 0,
          "Future bound overflow cannot wrap, mutate or consume a source byte");
}

void read_guards() {
  NandReadTiming read(ghz);
  read.observe(0, {.ce_n = true, .re_n = true});
  read.observe(selected, {.ce_n = false, .re_n = true});
  read.observe(initial_fall, {.ce_n = false, .re_n = false}, [] { return first; });
  const auto active = read;
  reject([&] { read.resume_ce_window(start, end, false); }, "RE high");
  require(read == active, "Active read cannot be retired");
  read.observe(initial_rise, {.ce_n = false, .re_n = true});
  const auto released = read;
  reject([&] { read.resume_ce_window(unexpired_release, end, false); }, "expired output release");
  require(read == released, "Release uncertainty cannot be dropped early");
  read.resume_ce_window(expired_release, end, false);
  require(read.output(end) == NandReadOutput{.driven = false},
          "Only proven expired release permits floating output");
}

void write_guards() {
  NandWriteTiming write(ghz);
  auto pins = write_pins();
  pins.ce_n = false;
  require(!write.observe(0, pins), "Initial selected controls have no latch");
  pins.we_n = false;
  require(!write.observe(initial_fall, pins), "Actual write pulse begins");
  const auto active = write;
  reject([&] { write.resume_ce_window(start, end, false); }, "WE/RE high");
  require(write == active, "Active write pulse cannot disappear");
  pins.we_n = true;
  require(write.observe(initial_rise, pins).has_value(),
          "Actual write latch occurs before hold checks");
  const auto latched = write;
  reject([&] { write.resume_ce_window(early_write_hold, end, false); }, "hold constraints");
  require(write == latched, "Unexpired real write hold cannot be retired");
  // A shorter preceding legal pulse makes the cycle bound, not hold, decisive.
  NandWriteTiming cycle(ghz);
  pins = write_pins();
  pins.ce_n = false;
  require(!cycle.observe(0, pins), "Cycle setup");
  pins.we_n = false;
  require(!cycle.observe(initial_fall, pins), "Cycle pulse starts");
  pins.we_n = true;
  constexpr unsigned shortest_pulse = 21;
  require(cycle.observe(initial_fall + shortest_pulse, pins).has_value(),
          "Independent minimum pulse");
  const auto pending = cycle;
  reject([&] { cycle.resume_ce_window(early_write_cycle, end, false); }, "pulse/hold constraints");
  require(cycle == pending, "Unexpired tWC remains strict");
  cycle.resume_ce_window(expired_write_cycle, end, false);
}

void setup_guards() {
  NandReadSetup setup(ghz);
  auto pins = setup_pins();
  require(!setup.observe(0, pins), "Initial setup level");
  const auto before = setup;
  reject([&] { setup.resume_ce_window(early_read_setup, end, false); }, "expired read setup");
  require(setup == before, "Unexpired tWHR cannot be dropped");
  pins.ready = false;
  require(!setup.observe(selected, pins), "Ready is actually withdrawn");
  const auto busy = setup;
  reject([&] { setup.resume_ce_window(start, end, false); }, "Ready high");
  require(setup == busy, "Unknown/busy-ready state cannot become qualified from a window");
}
}

void guards() {
  uninitialized_and_time();
  read_guards();
  write_guards();
  setup_guards();
  std::cout
      << "CE-window-guard-groups=4 uninitialized-active-hold-cycle-setup-time-overflow=strict\n";
}
}
