// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_omr_clock_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    using namespace blofeld::test::omr_clock;
    for (const auto source : sources) {
      for (const unsigned value : {0x305U, 0x389U, 0x600389U}) {
        one_case(
            {.source = source, .value = value, .opcode = direct_prefix | (source.field << 8U)});
      }
    }
    one_case(
        {.source = sources[0], .value = short_value, .opcode = short_prefix | (short_value << 8U)});
    one_case({.source = sources[0], .value = priority_omr, .opcode = long_opcode, .words = 2});
    restores();
    std::cout << "38 native clocked OMR streams and 14 restored suffixes with real delayed aliases "
                 "passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
