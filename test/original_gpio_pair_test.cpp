// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>

namespace {
using namespace blofeld::test::gpio_pair;
using namespace blofeld;
using blofeld::test::require;

int execute(const int argc, char **argv) {
  require(argc == 4, "probe archive report trace");
  const auto firmware = Firmware::load(argv[1]);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Private report files must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (unsigned which = 0; which < cases.size(); ++which) {
    for (const unsigned ratio : {0U, 1U}) {
      for (const unsigned phase : {0U, phase_offset}) {
        trace << "CASE " << which << ' ' << ratio << ' ' << phase << '\n';
        OriginalGpioPair pair(firmware, cases[which], trace);
        const auto nvm = pair.mcu.bus().power_off_flash_image();
        try {
          pair.run(ratio == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
                   phase);
          pair.verify_retained(firmware, nvm);
          require(pair.link.source_register_checks == 1, "Exactly one actual original source load");
          require(pair.link.pad_queries != 0 && pair.link.unresolved_queries == 0,
                  "Original consumers must sample only known net states");
          bool refused = false;
          try {
            pair.reject_board_qualification();
          } catch (const Fault &) { refused = true; }
          require(refused,
                  "Original helper return must not certify unresolved electrical intervals");
          report << "RETURN board-qualification=rejected nvm-unchanged=1 program-unchanged=1 ";
        } catch (const std::exception &e) {
          report << "FAIL ";
          report << e.what() << " | ";
        }
        report << which << ' ' << ratio << ' ' << phase << " mpc=" << hex(pair.mcu.cpu().state().pc)
               << " dpc=" << hex(pair.dsp.cpu().state().pc) << " ms=" << pair.link.msteps
               << " ds=" << pair.link.dsteps
               << " source-register-checks=" << pair.link.source_register_checks
               << " unresolved-events=" << pair.link.unresolved_events
               << " queries=" << pair.link.pad_queries
               << " unresolved-queries=" << pair.link.unresolved_queries
               << " value=" << pair.input.value << '\n';
        report.flush();
        trace.flush();
      }
    }
  }
  return 0;
}

}

int main(const int argc, char **argv) {
  try {
    return execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
