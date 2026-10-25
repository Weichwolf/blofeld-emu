// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "original_update_transport_fixture.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::update_transport;
using blofeld::test::require;

constexpr std::array keyboard_layouts{
    Layout{.mode = 3, .address = 0xef00, .declared = 42, .stored = 42, .packets = 6},
    Layout{.mode = 0, .address = 0x2e00, .declared = 63, .stored = 63, .packets = 9},
    Layout{.mode = 3, .address = 0xef00, .declared = 1542, .stored = 1542, .packets = 221},
    Layout{.mode = 0, .address = 0x2e00, .declared = 57, .stored = 58, .packets = 9}};
}

int main(const int argc, char **argv) {
  try {
    require(argc == 5,
            "Expected original main firmware, keyboard packets, own templates and trace");
    const auto firmware = Firmware::load(argv[1]);
    const auto blocks = read_blocks(argv[2], keyboard_layouts);
    const auto templates = read_templates(argv[3]);
    std::ofstream trace(argv[4]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (unsigned n = 0; n < keyboard_layouts.size(); ++n) {
      trace << "BLOCK index=" << n << '\n';
      Transfer run(firmware, blocks.at(n), keyboard_layouts.at(n), templates, trace);
      run.run();
    }
    std::cout << "Four original keyboard transfers and their complete padding stores verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
