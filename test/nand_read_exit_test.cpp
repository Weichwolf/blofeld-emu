// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned selected_fall = 100;
constexpr unsigned window_start = 200;
constexpr unsigned window_end = 300;
constexpr unsigned literal_early_control = 140;
constexpr unsigned idle_ce_release = 100;
constexpr unsigned idle_control_change = 101;
constexpr NandByte sample{.value = 0x5a, .known = 0xff};

struct Rate {
  unsigned hz;
  unsigned access;
  unsigned hold;
  unsigned write_pulse;
  unsigned write_hold;
};

// Independent literal ceilings of tREA30ns,tCSD10ns,tWP21ns and tCH/tCLH5ns.
constexpr std::array<Rate, 3> rates{
    {{.hz = 1000000000, .access = 30, .hold = 10, .write_pulse = 21, .write_hold = 5},
     {.hz = 1080000000, .access = 33, .hold = 11, .write_pulse = 23, .write_hold = 6},
     {.hz = 216000000, .access = 7, .hold = 3, .write_pulse = 5, .write_hold = 2}}};
unsigned assertions = 0;

void require(const bool value, const std::string_view message) {
  ++assertions;
  if (!value) { throw Fault(std::string(message)); }
}

void rejected(const auto &operation) {
  bool caught = false;
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains("tCSD"), error.what());
    caught = true;
  }
  require(caught, "Missing selected-read tCSD rejection");
}

NandReadSetupPins pins(const bool ce = false) {
  return {.ce_n = ce, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = true};
}

struct ReadPair {
  explicit ReadPair(const unsigned hz) : setup(hz), output(hz) {}

  NandReadSetup setup;
  NandReadTiming output;
  NandReadSetupPins controls = pins();
  unsigned sources = 0;
};

ReadPair selected_read(const Rate row, const bool rise_before, const NandReadKind kind) {
  ReadPair pair(row.hz);
  pair.controls.ready = kind == NandReadKind::array;
  require(!pair.setup.observe(0, pair.controls, kind), "Initial known pins are not a read");
  pair.output.observe(0, {.ce_n = false, .re_n = true});
  pair.controls.re_n = false;
  require(pair.setup.observe(selected_fall, pair.controls, kind),
          "An actual selected RE is admitted");
  pair.output.observe(selected_fall, {.ce_n = false, .re_n = false}, [&] {
    ++pair.sources;
    return sample;
  });
  const auto access = selected_fall + row.access;
  require(pair.output.output(access) == NandReadOutput{.driven = true, .data = sample} &&
              pair.sources == 1,
          "Independent access maximum produces exactly one genuine source byte");
  if (rise_before) {
    pair.controls.re_n = true;
    require(!pair.setup.observe(access, pair.controls, kind), "Actual RE rise consumes no byte");
    pair.output.observe(access, {.ce_n = false, .re_n = true});
  }
  return pair;
}

ReadPair read_exit(const Rate row, const bool rise_before, const NandReadKind kind) {
  auto pair = selected_read(row, rise_before, kind);
  const auto release = selected_fall + row.access + 1;
  pair.controls.ce_n = true;
  require(!pair.setup.observe(release, pair.controls, kind), "CE release is not a read");
  pair.output.observe(release, {.ce_n = true, .re_n = pair.controls.re_n});
  return pair;
}

void literal_countertrace() {
  // Source review6539096268: selected RE100,real access130,CE131,ACL140/141.
  auto pair = read_exit(rates[0], false, NandReadKind::array);
  const auto before = pair.setup;
  pair.controls.cle = true;
  pair.controls.ale = true;
  rejected([&] { static_cast<void>(pair.setup.observe(literal_early_control, pair.controls)); });
  require(pair.setup == before, "The literal9ns refusal preserves all read-setup state");
}

struct ControlClock {
  unsigned controls;
  unsigned clocks;
};

void boundary_case(const ReadPair &prefix,
                   const Rate row,
                   const NandReadKind kind,
                   const ControlClock choice) {
  const auto release = selected_fall + row.access + 1;
  auto pair = prefix;
  auto next = pair.controls;
  if (!next.re_n.has_value()) { throw Fault("Boundary suffix requires the actual known RE level"); }
  next.cle = (choice.controls & 1U) != 0;
  next.ale = (choice.controls & 2U) != 0;
  if ((choice.clocks & 1U) != 0) { next.re_n = !next.re_n.value(); }
  if ((choice.clocks & 2U) != 0) { next.we_n = false; }
  rejected([&] { static_cast<void>(pair.setup.observe(release + row.hold - 1, next, kind)); });
  require(pair.setup == prefix.setup, "Refusal preserves the full copied state");
  require(!pair.setup.observe(release + row.hold, next, kind), "Exact tCSD creates no read");
  pair.output.observe(release + row.hold, {.ce_n = true, .re_n = next.re_n}, [&] {
    ++pair.sources;
    return sample;
  });
  require(pair.sources == 1, "Unselected clocks never consume another byte");
  next.cle = false;
  next.ale = false;
  require(!pair.setup.observe(release + row.hold, next, kind),
          "ACL changes cannot restart an expired hold");
}

void boundary_suffixes() {
  for (const auto row : rates) {
    for (const bool rise_before : {false, true}) {
      for (const auto kind : {NandReadKind::array, NandReadKind::status70}) {
        const auto prefix = read_exit(row, rise_before, kind);
        for (const unsigned mask : {1U, 2U, 3U}) {
          for (unsigned clocks = 0; clocks < 4; ++clocks) {
            for (unsigned replay = 0; replay < 3; ++replay) {
              boundary_case(prefix, row, kind, {.controls = mask, .clocks = clocks});
            }
          }
        }
      }
    }
  }
}

void no_invented_read() {
  for (const bool ce : {false, true}) {
    for (const unsigned initial : {0U, 1000U}) {
      NandReadSetup setup(rates[0].hz);
      NandWriteTiming write(rates[0].hz);
      auto next = pins(ce);
      NandWritePins input{.ce_n = ce, .re_n = true, .we_n = true, .cle = false, .ale = false};
      require(!setup.observe(initial, next), "First pins do not manufacture a selected read");
      require(!write.observe(initial, input), "First pins do not manufacture a write");
      next.ce_n = true;
      input.ce_n = true;
      require(!setup.observe(initial + idle_ce_release, next), "CE rise without a preceding read");
      require(!write.observe(initial + idle_ce_release, input),
              "CE rise without a preceding latch");
      next.cle = true;
      next.ale = true;
      input.cle = true;
      input.ale = true;
      require(!setup.observe(initial + idle_control_change, next),
              "No universal read-exit hold is invented");
      require(!write.observe(initial + idle_control_change, input),
              "Read-exit admission does not change the write contract");
    }
  }
}

void write_holds() {
  for (const auto row : rates) {
    NandReadSetup setup(row.hz);
    NandWriteTiming write(row.hz);
    auto next = pins();
    next.cle = true;
    NandWritePins input{
        .ce_n = false, .re_n = true, .we_n = true, .cle = true, .ale = false, .data = sample};
    require(!setup.observe(0, next), "Write-mode setup has no selected RE");
    require(!write.observe(0, input), "Known command and data setup");
    next.we_n = false;
    input.we_n = false;
    require(!setup.observe(selected_fall, next), "Falling WE is not an RE event");
    require(!write.observe(selected_fall, input), "Real selected write pulse begins");
    const auto latch = selected_fall + row.write_pulse;
    next.we_n = true;
    input.we_n = true;
    require(!setup.observe(latch, next), "Rising WE is not an RE event");
    require(write.observe(latch, input) ==
                NandWriteLatch{.kind = NandWriteKind::command, .value = sample.value},
            "A real command latches at the independent tWP minimum");
    next.ce_n = true;
    next.cle = false;
    input.ce_n = true;
    input.cle = false;
    require(!setup.observe(latch + row.write_hold, next),
            "No selected read means no new read-exit guard");
    require(!write.observe(latch + row.write_hold, input),
            "Existing independent5ns CE/CLE write holds remain unchanged");
  }
}

void opaque_windows() {
  for (const auto row : rates) {
    for (const bool ce : {false, true}) {
      auto pair = selected_read(row, true, NandReadKind::array);
      pair.setup.resume_ce_window(window_start, window_end, ce);
      pair.controls.ce_n = ce;
      const auto before = pair.setup;
      pair.controls.cle = true;
      rejected(
          [&] { static_cast<void>(pair.setup.observe(window_end + row.hold - 1, pair.controls)); });
      require(pair.setup == before,
              "Opaque possible read exit is bounded without fabricating an edge");
      require(!pair.setup.observe(window_end + row.hold, pair.controls),
              "Latest possible read-exit bound expires exactly");
      pair.controls.cle = false;
      require(!pair.setup.observe(window_end + row.hold, pair.controls),
              "ACL-only change does not restart the possible-history bound");
      NandReadSetup idle(row.hz);
      auto next = pins();
      require(!idle.observe(0, next), "No observed read before the CE window");
      idle.resume_ce_window(window_start, window_end, ce);
      next.ce_n = ce;
      next.ale = true;
      require(!idle.observe(window_end + 1, next),
              "Opaque CE alone cannot manufacture a selected RE");
    }
  }
}

void pending_window() {
  for (const auto row : rates) {
    auto pair = read_exit(row, true, NandReadKind::array);
    const auto release = selected_fall + row.access + 1;
    const auto before = pair.setup;
    rejected([&] { pair.setup.qualify_ce_window(release + row.hold - 1); });
    require(pair.setup == before, "Window qualification cannot erase a pending actual read exit");
    pair.setup.qualify_ce_window(release + row.hold);
    require(pair.setup == before, "Qualification at the exact bound is observational");
  }
}
}

int main(const int argc, char **argv) {
  try {
    const bool countertrace = argc == 2 && std::string_view(argv[1]) == "--countertrace-only";
    require(argc == 1 || countertrace, "Unknown test argument");
    literal_countertrace();
    if (!countertrace) {
      boundary_suffixes();
      no_invented_read();
      write_holds();
      opaque_windows();
      pending_window();
    }
    std::cout << "Selected-read exit:3 literal rates,array/status70,RE-low/high,ACL/clock/copy "
                 "boundaries,opaque history and unchanged write holds;assertions="
              << assertions << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
