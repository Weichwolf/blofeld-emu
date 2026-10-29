// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "nand_ce_window_fixture.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace blofeld::test::ce_window {
namespace {
constexpr unsigned ce_column = 1;
constexpr unsigned re_column = 2;
constexpr unsigned cs_column = 3;
constexpr unsigned whr_column = 5;
constexpr unsigned clr_column = 6;
constexpr unsigned rr_column = 7;
constexpr unsigned hold_column = 9;
constexpr std::uint64_t actual_selection = 650;
constexpr std::uint64_t became_unstable = 700;
constexpr std::uint64_t became_stable = 800;

void read_bounds(const std::array<unsigned, 10> &row, const bool close_high) {
  auto timing = read_base(row[0]);
  timing.resume_ce_window(start, end, close_high);
  const auto saved = timing;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    auto restored = saved;
    const auto selection = close_high ? actual_selection : end;
    if (close_high) { restored.observe(selection, {.ce_n = false, .re_n = true}); }
    unsigned calls = 0;
    restored.observe(selection + 1, {.ce_n = false, .re_n = false}, [&] {
      ++calls;
      return second;
    });
    const auto deadline = selection + row[ce_column];
    require(row[ce_column] >= 1 + row[re_column], "Independent vectors include both access maxima");
    require(restored.output(deadline - 1) == NandReadOutput{},
            "No byte before the latest possible CE access bound");
    require(restored.output(deadline) == NandReadOutput{.driven = true, .data = second} &&
                calls == 1,
            "Exact independent tCEA bound yields one actual source byte");
    restored.observe(deadline, {.ce_n = false, .re_n = false}, [&] {
      ++calls;
      return first;
    });
    require(calls == 1, "Repeated known levels cannot replay the source");
  }
}

void write_bounds(const std::array<unsigned, 10> &row, const bool close_high) {
  auto timing = write_base(row[0]);
  timing.resume_ce_window(start, end, close_high);
  auto pins = write_pins();
  pins.ce_n = false;
  const auto selection = close_high ? actual_selection : end;
  if (close_high) {
    require(!timing.observe(selection, pins), "Real following CE fall has no latch");
  }
  pins.we_n = false;
  require(!timing.observe(selection, pins), "First real WE fall cannot latch data");
  const auto pending = timing;
  pins.we_n = true;
  reject([&] { static_cast<void>(timing.observe(selection + row[cs_column] - 1, pins)); }, "tCS");
  require(timing == pending, "Early tCS failure preserves the complete pending state");
  const auto edge = selection + row[cs_column];
  require(timing.observe(edge, pins) ==
              NandWriteLatch{.kind = NandWriteKind::command, .value = first.value},
          "Exact independent tCS boundary admits the real command");
  const auto latched = timing;
  pins.data = second;
  reject([&] { static_cast<void>(timing.observe(edge + row[hold_column] - 1, pins)); }, "tDH");
  require(timing == latched && !timing.observe(edge + row[hold_column], pins),
          "Real post-window latch reinstalls exact data hold and admits its boundary");
}

void change_setup(NandReadSetupPins &pins, const unsigned field, const bool unstable) {
  switch (field) {
    case 0: pins.we_n = !unstable; break;
    case 1: pins.ale = unstable; break;
    case 2: pins.cle = unstable; break;
    case 3: pins.ready = !unstable; break;
    default: throw Fault("Invalid independent read setup field");
  }
}

void reinstated_setup(const std::array<unsigned, 10> &row, const unsigned field) {
  constexpr std::array<unsigned, 4> columns{whr_column, clr_column, clr_column, rr_column};
  constexpr std::array<const char *, 4> names{"tWHR", "tAR", "tCLR", "tRR"};
  NandReadSetup timing(row[0]);
  auto pins = setup_pins();
  require(!timing.observe(0, pins), "Initial setup is not a read");
  timing.resume_ce_window(start, end, false);
  pins.ce_n = false;
  change_setup(pins, field, true);
  require(!timing.observe(became_unstable, pins), "A setup change alone cannot read");
  change_setup(pins, field, false);
  require(!timing.observe(became_stable, pins),
          "A real stable transition reinstalls its own anchor");
  const auto before = timing;
  const auto edge = became_stable + row[columns[field]];
  pins.re_n = false;
  reject([&] { static_cast<void>(timing.observe(edge - 1, pins)); }, names[field]);
  require(timing == before && timing.observe(edge, pins),
          "Exact independently rounded setup bound admits once");
}
}

void boundaries() {
  for (const auto &row : vectors) {
    for (const bool close_high : {false, true}) {
      read_bounds(row, close_high);
      write_bounds(row, close_high);
    }
    for (unsigned field = 0; field < 4; ++field) { reinstated_setup(row, field); }
  }
  std::cout << "CE-window-boundaries frequencies=3 closing-levels=2 read-write-copy-suffixes=12 "
               "reinstated-setup-cases=12\n";
}
}
