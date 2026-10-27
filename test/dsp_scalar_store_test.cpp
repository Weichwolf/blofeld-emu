// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld::test;
using namespace blofeld::test::scalar_store;
constexpr std::array<std::uint64_t, 18> values{0,
                                               1,
                                               0x007fffffffffff,
                                               0x00800000000000,
                                               0x00ffffffffffff,
                                               0xff800000000000,
                                               0xff7fffffffffff,
                                               0xffffffffffffff,
                                               0x00200000000000,
                                               0x00400000000000,
                                               0x00600000000000,
                                               0x00800000000001,
                                               0x7fffffffffffff,
                                               0x80000000000000,
                                               0x00000000ffffff,
                                               0xffffffff000001,
                                               0xffffffffff0000,
                                               0x00000001000000};
constexpr std::array initial_flags{0U,
                                   blofeld::test::sticky | 0x3fU,
                                   blofeld::test::limit | 0x15U,
                                   blofeld::test::sticky | blofeld::test::limit | 0x2aU};

unsigned addresses(Case sample) {
  unsigned vectors = 0;
  for (const unsigned flags : initial_flags) {
    for (const unsigned reg : {0U, 7U}) {
      for (unsigned mode = 0; mode <= 4; ++mode) {
        sample.flags = flags;
        sample.reg = reg;
        sample.mode = mode;
        one(sample, false);
        ++vectors;
      }
    }
  }
  return vectors;
}

unsigned matrix() {
  unsigned vectors = 0;
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const auto value : values) {
          vectors += addresses({.value = value, .scale = scale, .b = b, .y = y});
        }
      }
    }
  }
  return vectors;
}
}

int main(const int count, char **arguments) {
  try {
    using namespace blofeld::test::scalar_store;
    const std::string_view mode = count == 2 ? arguments[1] : "--matrix";
    const bool rejection = mode == "--expect-refusal";
    if (count > 2 || (!rejection && mode != "--matrix" && mode != "--smoke")) {
      throw blofeld::Fault("Usage: scalar-store-test [--expect-refusal|--smoke|--matrix]");
    }
    constexpr std::uint64_t smoke_value = 0x123456789abc;
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) { one({.value = smoke_value, .b = b, .y = y}, rejection); }
    }
    if (mode != "--matrix") {
      std::cout << (rejection ? "Four current unsupported scalar-store cases reproduced\n"
                              : "Four native full-accumulator scalar stores passed\n");
      return 0;
    }
    const auto vectors = matrix();
    restores();
    guards();
    std::cout << "84 snapshot cuts with two literal replays, 9 form refusals, 4 reserved-scale "
                 "cuts passed\n";
    std::cout << vectors << " native full A/B scalar-store scale/clip/flag/AGU streams passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
