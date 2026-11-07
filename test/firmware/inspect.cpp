// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"

#include <exception>
#include <iostream>
#include <string_view>

int main(const int argc, char **argv) {
  try {
    if (argc != 4 || std::string_view(argv[1]) != "--firmware" ||
        std::string_view(argv[3]) != "--inspect") {
      throw blofeld::Fault("Usage: firmware-parser-test --firmware input --inspect");
    }
    const auto firmware = blofeld::Firmware::load(argv[2]);
    std::cout << firmware.mcu.size() << " MCU bytes, " << firmware.dsp.size() << " DSP records, "
              << firmware.sysex_packets << " checked data packets\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
