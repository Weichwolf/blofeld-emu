// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_modifier_move_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld::test;
using namespace blofeld::test::modifier_move;
constexpr std::array values{0U, 1U, 0xffU, 0x012345U, 0x7fffffU, 0x800000U, 0xffbfffU, 0xffffffU};
constexpr std::array flags{0U, 0x3fU, 0xc0U, 0xffU};

unsigned modes(Case sample) {
  unsigned vectors = 0;
  for (const unsigned scale : {0U, scale_down, scale_up}) {
    for (const auto ccr : flags) {
      sample.scale = scale;
      sample.flags = ccr;
      one(sample);
      ++vectors;
    }
  }
  return vectors;
}

unsigned matrix() {
  unsigned vectors = 0;
  for (unsigned source = 0; source < register_count; ++source) {
    for (unsigned destination = 0; destination < destination_count; ++destination) {
      for (const auto value : values) {
        vectors += modes({.source = source, .destination = destination, .value = value});
      }
    }
  }
  return vectors;
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--matrix";
    if (count > 2 || (mode != "--matrix" && mode != "--smoke" && mode != "--expect-refusal")) {
      throw blofeld::Fault("Usage: modifier-move-test [--matrix|--smoke|--expect-refusal]");
    }
    constexpr unsigned cookie = 0x12345;
    constexpr std::array examples{Case{.source = 0, .destination = 1, .value = cookie},
                                  Case{.source = 1, .destination = 8, .value = cookie},
                                  Case{.source = 7, .destination = 9, .value = cookie},
                                  Case{.source = 7, .destination = 0, .value = cookie}};
    for (const auto sample : examples) {
      if (mode == "--expect-refusal") {
        refusal(sample);
      } else {
        one(sample);
      }
    }
    if (mode != "--matrix") {
      std::cout << "Four native modifier transfer observations completed\n";
      return 0;
    }
    const auto vectors = matrix();
    restores();
    guards();
    std::cout << vectors
              << " raw modifier streams,84 snapshot cuts and14 strict guards "
                 "with two replays passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
