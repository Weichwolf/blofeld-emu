// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_repeat_clock_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    unsigned suffixes = 0;
    for (const unsigned count : {1U, 2U, 3U, 17U}) {
      for (const bool memory : {false, true}) {
        for (const bool reg : {false, true}) {
          suffixes += blofeld::test::repeat_clock::one({.count = count,
                                                        .memory = memory,
                                                        .reg = reg,
                                                        .mutate = false,
                                                        .checkpoints = false});
        }
      }
    }
    std::cout << "16 automatic REP streams, " << suffixes << " joint checkpoint suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
