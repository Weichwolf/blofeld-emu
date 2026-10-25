// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_peripheral_pipeline_fixture.hpp"

#include <exception>
#include <iostream>
#include <string_view>

int main(const int argc, char **argv) {
  try {
    using namespace blofeld::test::peripheral_pipeline;
    if (argc == 2 && std::string_view(argv[1]) == "writes") {
      writes();
    } else if (argc == 2 && std::string_view(argv[1]) == "state") {
      state_checks();
    } else if (argc == 2 && std::string_view(argv[1]) == "register-writes") {
      register_writes();
    } else if (argc == 2 && std::string_view(argv[1]) == "register-state") {
      register_state_checks();
    } else if (argc == 4 && std::string_view(argv[1]) == "bootstrap") {
      bootstrap({.rom = argv[2], .trace = argv[3]});
    } else {
      throw blofeld::Fault(
          "Expected writes, state, register-writes, register-state or bootstrap ROM TRACE");
    }
    std::cout << "Native peripheral pipeline checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
