// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "nand_ce_window_fixture.hpp"

#include <exception>
#include <iostream>
#include <string>

int main(const int argc, char **argv) {
  try {
    blofeld::test::ce_window::require(argc == 2, "Supply boundaries, guards or histories");
    const std::string mode = argv[1];
    if (mode == "boundaries") {
      blofeld::test::ce_window::boundaries();
    } else if (mode == "guards") {
      blofeld::test::ce_window::guards();
    } else if (mode == "histories") {
      blofeld::test::ce_window::histories();
    } else {
      throw blofeld::Fault("Unknown CE-window check");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
