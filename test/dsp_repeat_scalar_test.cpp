// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_scalar_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld::test;
using namespace blofeld::test::repeat_scalar;
constexpr std::array<std::uint64_t, 3> values{0x00200000000000, 0x007fffffffffff, 0x80000000000000};

unsigned addresses(Case sample) {
  unsigned vectors = 0;
  for (const unsigned reg : {0U, 7U}) {
    for (unsigned mode = 0; mode <= 4; ++mode) {
      sample.store.reg = reg;
      sample.store.mode = mode;
      one(sample);
      ++vectors;
    }
  }
  return vectors;
}

unsigned scalings(Case sample) {
  unsigned vectors = 0;
  for (const auto value : values) {
    for (const unsigned scale : {0U, scale_down, scale_up}) {
      sample.store.value = value;
      sample.store.scale = scale;
      vectors += addresses(sample);
    }
  }
  return vectors;
}

unsigned matrix() {
  unsigned vectors = 0;
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (const bool reg : {false, true}) {
        for (const unsigned count : {1U, 3U, 17U}) {
          vectors += scalings({.count = count, .reg = reg, .store = {.b = b, .y = y}});
        }
      }
    }
  }
  return vectors;
}

unsigned suffixes() {
  unsigned cuts = 0;
  constexpr std::uint64_t clipped_value = 0x00800000000001;
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (const bool reg : {false, true}) {
        cuts += one({.count = 3,
                     .reg = reg,
                     .mutate = true,
                     .store = {.value = clipped_value, .scale = scale_up, .b = b, .y = y}},
                    true);
      }
    }
  }
  return cuts;
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--matrix";
    if (count > 2 || (mode != "--matrix" && mode != "--expect-refusal" && mode != "--smoke")) {
      throw blofeld::Fault("Usage: repeat-scalar-test [--expect-refusal|--smoke|--matrix]");
    }
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        const Case sample{.reg = true, .store = {.value = values[0], .b = b, .y = y}};
        if (mode == "--expect-refusal") {
          refused(sample);
        } else {
          one(sample);
        }
      }
    }
    if (mode != "--matrix") {
      std::cout << "Four repeated A/B->X/Y scalar observations completed\n";
      return 0;
    }
    const auto vectors = matrix();
    const auto cuts = suffixes();
    // Own stream with the stored workspace dimensions; not original reset/boot.
    constexpr unsigned workspace = 0x936;
    constexpr unsigned workspace_words = 0x1ac0;
    one({.count = workspace_words, .reg = true, .start = workspace});
    std::cout << vectors << " REP scalar streams," << cuts
              << " snapshot cuts with two replays,8 latch mutations,6848-word workspace passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
