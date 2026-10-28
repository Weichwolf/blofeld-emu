// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_accumulator_preupdate_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld::test;
namespace form = repeat_accumulator;
constexpr unsigned expected_streams = 192;
// FM13-119/123/Table12-14: literal X/Y accumulator fields, R0 and modes5/7.
constexpr std::array x_indexed{
    0x506800U, 0x516800U, 0x526800U, 0x536800U, 0x546800U, 0x556800U, 0x566800U, 0x576800U};
constexpr std::array x_predecrement{
    0x507800U, 0x517800U, 0x527800U, 0x537800U, 0x547800U, 0x557800U, 0x567800U, 0x577800U};
constexpr std::array y_indexed{
    0x586800U, 0x596800U, 0x5a6800U, 0x5b6800U, 0x5c6800U, 0x5d6800U, 0x5e6800U, 0x5f6800U};
constexpr std::array y_predecrement{
    0x587800U, 0x597800U, 0x5a7800U, 0x5b7800U, 0x5c7800U, 0x5d7800U, 0x5e7800U, 0x5f7800U};

unsigned literal(const form::Case sample) {
  const auto index = sample.source - 8;
  if (sample.mode == 5) { return (sample.y ? y_indexed : x_indexed).at(index); }
  return (sample.y ? y_predecrement : x_predecrement).at(index);
}

struct Stats {
  unsigned streams = 0, cuts = 0;
};

void source(const unsigned field, const bool snapshots, Stats &stats) {
  for (const unsigned mode : {5U, 7U}) {
    for (const bool y : {false, true}) {
      for (const auto counter : {form::Counter::immediate, form::Counter::x0}) {
        for (const unsigned count : {1U, 3U, 17U}) {
          const form::Case sample{.source = field,
                                  .mode = mode,
                                  .count = count,
                                  .counter = counter,
                                  .y = y,
                                  .mutate = snapshots};
          require(form::opcode(sample) == literal(sample),
                  "Every accumulator source/space/mode retains its independent literal encoding");
          stats.cuts += form::one(sample, snapshots);
          ++stats.streams;
        }
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--streams";
    if (count > 2 || (mode != "--streams" && mode != "--snapshots")) {
      throw blofeld::Fault("Expected --streams or --snapshots");
    }
    Stats stats;
    for (unsigned field = 8; field <= 15; ++field) { source(field, mode == "--snapshots", stats); }
    require(stats.streams == expected_streams,
            "All eight accumulator fields, spaces, addressing modes and counts execute");
    std::cout << stats.streams << " accumulator REP preupdate streams passed " << mode
              << "; cuts=" << stats.cuts << " suffixes=" << 2 * stats.cuts << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
