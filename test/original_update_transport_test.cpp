// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "original_update_transport_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>

using namespace blofeld;
using namespace blofeld::test::update_transport;
using blofeld::test::require;

int main(const int argc, char **argv) {
  try {
    require(argc == 5, "Expected original firmware, private packets, own templates and trace");
    const auto firmware = Firmware::load(argv[1]);
    const auto blocks = read_blocks(argv[2]);
    const auto templates = read_templates(argv[3]);
    std::ofstream trace(argv[4]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (unsigned n = 0; n < layouts.size(); ++n) {
      trace << "BLOCK index=" << n << '\n';
      Transfer run(firmware, blocks.at(n), layouts.at(n), templates, trace);
      run.run();
    }
    std::cout << "Six original update transfers and their complete padding stores verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
