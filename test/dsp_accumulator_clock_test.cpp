// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_accumulator_clock_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  using namespace blofeld::test::accumulator_clock;
  try {
    for (const auto source : {Source::a, Source::b}) {
      for (const auto target : destinations) {
        for (unsigned scale = 0; scale < 3; ++scale) {
          for (const auto value : values) {
            for (const bool initial_growth : {false, true}) {
              one_case({.source = source,
                        .destination = target,
                        .scale = scale,
                        .value = value,
                        .sticky = initial_growth});
            }
          }
        }
      }
    }
    std::cout << "1152 literal full-state accumulator streams passed\n";
    restores();
    std::cout << "84 actual trusted suffixes passed\n";
    guards();
    reserved_scaling();
    std::cout
        << "1152 native accumulator streams,84 trusted suffixes and5 strict refusals passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
