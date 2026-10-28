// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_partial_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld::test;
using namespace blofeld::test::repeat_partial;
constexpr std::array<std::uint64_t, 3> values{0x7f7fffff123456, 0x80800000654321, 0xffffffffffabcd};

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
          for (unsigned part = 0; part < 3; ++part) {
            vectors += scalings(
                {.count = count, .reg = reg, .store = {.b = b, .y = y}, .source_part = part});
          }
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
        for (unsigned part = 0; part < 3; ++part) {
          cuts += one({.count = 3,
                       .reg = reg,
                       .mutate = true,
                       .store = {.value = clipped_value, .scale = scale_up, .b = b, .y = y},
                       .source_part = part},
                      true);
        }
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
    unsigned fills = 0;
    constexpr unsigned prefix_words = 77;
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        for (const bool reg : {false, true}) {
          one({.count = prefix_words,
               .reg = reg,
               .store = {.value = modifier_output, .b = b, .y = y},
               .source_part = 1,
               .modifier_source = true});
          ++fills;
        }
      }
    }
    std::cout << vectors << " REP partial streams," << cuts
              << " cuts/two suffixes,24 body-latch mutations," << fills
              << " own native M1->A/B plus77-word prefix fills passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
