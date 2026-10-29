// SPDX-License-Identifier: GPL-3.0-only
#include "nand_ce_window_fixture.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace blofeld::test::ce_window {
namespace {
constexpr unsigned patterns = 64;
constexpr unsigned samples = 6;
constexpr std::uint64_t quantum = 10;
constexpr std::uint64_t read_edge = end + 1;
constexpr std::uint64_t release_edge = end + 50;
constexpr std::uint64_t write_edge = end + 31;
constexpr std::array<std::uint64_t, 7> read_ticks{601, 630, 631, 634, 635, 640, 649};
constexpr std::array<std::uint64_t, 5> release_ticks{650, 664, 665, 679, 680};

void history(const unsigned pattern) {
  auto exact_read = read_base();
  auto bounded_read = read_base();
  auto exact_write = write_base();
  auto bounded_write = write_base();
  for (unsigned index = 0; index < samples; ++index) {
    const bool ce = (pattern & (1U << index)) != 0;
    const auto tick = start + (index * quantum);
    exact_read.observe(tick, {.ce_n = ce, .re_n = true});
    auto pins = write_pins();
    pins.ce_n = ce;
    require(!exact_write.observe(tick, pins), "CE history cannot create a WE latch");
  }
  exact_read.observe(end, {.ce_n = false, .re_n = true});
  auto pins = write_pins();
  pins.ce_n = false;
  require(!exact_write.observe(end, pins), "Known closing CE has no latch");
  bounded_read.resume_ce_window(start, end, false);
  bounded_write.resume_ce_window(start, end, false);
  exact_read.observe(read_edge, {.ce_n = false, .re_n = false}, [] { return second; });
  bounded_read.observe(read_edge, {.ce_n = false, .re_n = false}, [] { return second; });
  for (const auto tick : read_ticks) {
    const auto bounded = bounded_read.output(tick);
    const auto exact = exact_read.output(tick);
    if (bounded.driven.has_value()) {
      require(bounded == exact,
              "A newly known byte is correct in every directed concrete edge history");
    }
  }
  exact_read.observe(release_edge, {.ce_n = false, .re_n = true});
  bounded_read.observe(release_edge, {.ce_n = false, .re_n = true});
  for (const auto tick : release_ticks) {
    require(bounded_read.output(tick) == exact_read.output(tick),
            "Actual release suffix agrees after valid data");
  }
  pins.we_n = false;
  require(!exact_write.observe(end, pins) && !bounded_write.observe(end, pins),
          "Actual WE falling edges");
  pins.we_n = true;
  const auto actual = exact_write.observe(write_edge, pins);
  const auto bounded = bounded_write.observe(write_edge, pins);
  require(actual == NandWriteLatch{.kind = NandWriteKind::command, .value = first.value} &&
              bounded == actual,
          "Independent tCS31 admits the same one command for all directed histories");
}
}

void histories() {
  for (unsigned pattern = 0; pattern < patterns; ++pattern) { history(pattern); }
  std::cout << "concrete-CE-histories=64 read-output-relations=768 actual-command-comparisons=64\n";
}
}
