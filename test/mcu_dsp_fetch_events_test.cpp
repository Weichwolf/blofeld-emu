// SPDX-License-Identifier: GPL-3.0-only
#include "mcu_dsp_fetch_events_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    blofeld::test::joint_events::guard_cases();
    blofeld::test::joint_events::order_cases();
    blofeld::test::joint_events::early_events();
    std::cout << "Native MCU/dsp56300 event order, causality and refusal checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
