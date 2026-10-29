// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::ce_window {
inline constexpr unsigned ghz = 1000000000;
inline constexpr std::uint64_t selected = 100;
inline constexpr std::uint64_t initial_fall = 200;
inline constexpr std::uint64_t initial_rise = 280;
inline constexpr std::uint64_t deselected = 350;
inline constexpr std::uint64_t start = 500;
inline constexpr std::uint64_t end = 600;
inline constexpr NandByte first{.value = 0x96, .known = 0xff};
inline constexpr NandByte second{.value = 0x29, .known = 0xff};
// Primary-table rounding, independently fixed: Hz, tCEA, tREA, tCS, tWP,
// tWHR, tAR/tCLR, tRR, tOH/tWH and tCH/tDH in reference ticks.
inline constexpr std::array<std::array<unsigned, 10>, 3> vectors{
    {{{1000000000, 35, 30, 31, 21, 60, 10, 20, 15, 5}},
     {{1080000000, 38, 33, 34, 23, 65, 11, 22, 17, 6}},
     {{216000000, 8, 7, 7, 5, 13, 3, 5, 4, 2}}}};

inline void require(const bool value, const std::string &message) {
  if (!value) { throw Fault(message); }
}

inline void reject(const auto &action, const std::string &reason) {
  try {
    action();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(reason),
            "Wrong CE-window refusal: " + std::string(error.what()));
    return;
  }
  throw Fault("Missing CE-window refusal: " + reason);
}

inline NandReadTiming read_base(const unsigned hz = ghz) {
  NandReadTiming timing(hz);
  timing.observe(0, {.ce_n = true, .re_n = true});
  timing.observe(selected, {.ce_n = false, .re_n = true});
  timing.observe(initial_fall, {.ce_n = false, .re_n = false}, [] { return first; });
  timing.observe(initial_rise, {.ce_n = false, .re_n = true});
  timing.observe(deselected, {.ce_n = true, .re_n = true});
  return timing;
}

inline NandWritePins write_pins() {
  return {.ce_n = true, .re_n = true, .we_n = true, .cle = true, .ale = false, .data = first};
}

inline NandWriteTiming write_base(const unsigned hz = ghz) {
  NandWriteTiming timing(hz);
  auto pins = write_pins();
  require(!timing.observe(0, pins), "Initial WE high has no latch");
  pins.ce_n = false;
  require(!timing.observe(selected, pins), "CE selection has no latch");
  pins.we_n = false;
  require(!timing.observe(initial_fall, pins), "WE fall has no latch");
  pins.we_n = true;
  require(timing.observe(initial_rise, pins) ==
              NandWriteLatch{.kind = NandWriteKind::command, .value = first.value},
          "Initial actual command establishes pulse and hold history");
  pins.ce_n = true;
  require(!timing.observe(deselected, pins), "CE release has no latch");
  return timing;
}

inline NandReadSetupPins setup_pins() {
  return {.ce_n = true, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = true};
}

void boundaries();
void guards();
void histories();
}
