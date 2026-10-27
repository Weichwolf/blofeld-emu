// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_long_store_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    using namespace blofeld::test;

    unsigned vectors = 0;
    for (const bool b : {false, true}) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const auto value : long_saturation::values) {
          for (const unsigned flags : {0U, sticky | limit}) {
            for (const unsigned reg : {0U, 7U}) {
              for (unsigned mode = 0; mode <= 4; ++mode) {
                long_saturation::one({.value = value,
                                      .scale = scale,
                                      .flags = flags,
                                      .mode = mode,
                                      .reg = reg,
                                      .b = b});
                ++vectors;
              }
            }
          }
        }
      }
    }
    std::cout << vectors << " actual full A/B long-store scale/clip/flag/AGU streams passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
