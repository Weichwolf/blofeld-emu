// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_branch_pipeline_fixture.hpp"

#include <exception>
#include <iostream>
#include <string_view>

int main(const int argc, char **argv) {
  try {
    using namespace blofeld::test::branch_pipeline;
    if (argc == 2 && std::string_view(argv[1]) == "branches") {
      branches();
    } else if (argc == 2 && std::string_view(argv[1]) == "state") {
      state_checks();
    } else {
      throw blofeld::Fault("Expected branches or state");
    }
    std::cout << "Native OMR branch pipeline checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
