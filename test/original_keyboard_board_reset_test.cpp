// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard_board_reset_fixture.hpp"
#include "keyboard_receiver_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <sstream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_board_reset;
using blofeld::test::hcs08::require;

void run_case(const Firmware &firmware,
              const std::vector<std::uint8_t> &helper,
              const unsigned phase,
              const unsigned prescale,
              std::ostream &trace) {
  std::ostringstream events;
  Fixture run(firmware, helper, phase, prescale, events);
  trace << "CASE phase=" << phase << " prescale=" << prescale << '\n';
  try {
    while (run.host.cpu().state().pc != sync_request) {
      require(run.steps < step_limit, "Original reset/idle prefix reaches its actual SYNC request");
      run.step();
    }
    const auto saved = run.checkpoint();
    const auto offset = static_cast<std::size_t>(events.tellp());
    run.finish(prescale);
    const auto complete = events.str();
    const auto suffix = complete.substr(offset);
    trace << complete;
    for (unsigned n = 0; n < 2; ++n) {
      std::ostringstream actual;
      Fixture restored(firmware, helper, phase, prescale, actual);
      restored.restore(saved);
      restored.finish(prescale);
      require(actual.str() == suffix && restored.endpoint() == run.endpoint() &&
                  restored.target_owner().memory().flash_image() ==
                      run.target_owner().memory().flash_image() &&
                  restored.target_owner().memory().flash_state() ==
                      run.target_owner().memory().flash_state() &&
                  restored.target_owner().cpu_state() == run.target_owner().cpu_state() &&
                  restored.target_owner().bdc_state() == run.target_owner().bdc_state() &&
                  restored.target_owner().wire_state() == run.target_owner().wire_state() &&
                  restored.reset_sequence().state() == run.reset_sequence().state(),
              "Fresh MCU/target/native/reset/BDC/wire/TIM owners reproduce the entire SYNC suffix");
      trace << "REPLAY index=" << n << " bytes=" << actual.str().size() << " identical=1\n";
    }
    trace << "CASE_END phase=" << phase << " prescale=" << prescale << " steps=" << run.steps
          << '\n';
  } catch (...) {
    trace << events.str();
    throw;
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected original MCU firmware, complete HC12 helper and private trace");
    const auto firmware = Firmware::load(arguments[1]);
    const auto helper = blofeld::test::keyboard_receiver::bytes(arguments[2], helper_bytes);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    run_case(firmware, helper, 1, 2, trace);
    run_case(firmware, helper, 14, 2, trace);
    run_case(firmware, helper, 1, 6, trace);
    trace << "END cases=3 replays=6 accepted=2 rejected=1 target-retired=0\n";
    std::cout
        << "Original HC12 reset/SYNC uses production board memory,CPU,BDC and wire ownership.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
