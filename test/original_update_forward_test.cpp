// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>

using namespace blofeld;
using namespace blofeld::test::update_forward;
using blofeld::test::require;

int main(const int argc, char **argv) {
  try {
    require(argc == 3, "Original update forwarding needs firmware and private trace");
    const auto firmware = Firmware::load(argv[1]);
    std::ofstream trace(argv[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, phase_offset}) {
        trace << "CASE order=" << order << " phase=" << phase << '\n';
        Forward run(firmware, trace);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.execute(order, phase);
        run.verify(nvm);
      }
    }
    std::cout << "Original DSP/MCU error helper: four native IRQ/transfer/call contexts.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
