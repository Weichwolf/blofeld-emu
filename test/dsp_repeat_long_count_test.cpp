// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_repeat_clock_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    constexpr unsigned count = 131072;
    blofeld::test::repeat_clock::one({.count = count, .reg = true});
    std::cout << "131072 actual clocked native REP bodies passed full CPU/F1/F2/timing checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
