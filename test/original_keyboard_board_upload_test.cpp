// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard_board_transport_fixture.hpp"
#include "keyboard_board_upload_fixture.hpp"
#include "keyboard_receiver_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_board_upload;
using blofeld::test::hcs08::require;
constexpr unsigned record_count = 7, snapshot_count = 2;

struct Inputs {
  const Firmware &firmware;
  const std::vector<std::uint8_t> &helper;
  const std::vector<std::uint8_t> &records;
};

void run_case(const Inputs input, const unsigned phase, const bool secure, std::ostream &trace) {
  std::ostringstream events;
  Runner run(input.firmware, input.helper, phase, secure, events);
  trace << "CASE phase=" << phase << " secure=" << secure << '\n';
  try {
    run.prepare();
    run.buffer(input.records);
    run.start(input.records);
  } catch (...) {
    trace << events.str();
    throw;
  }
  const auto saved = run.checkpoint();
  const auto offset = static_cast<std::size_t>(events.tellp());
  run.detailed = true;
  try {
    run.until_fault();
  } catch (...) {
    trace << events.str();
    throw;
  }
  const auto complete = events.str();
  const auto suffix = complete.substr(offset);
  trace << complete;
  for (unsigned replay = 0; replay < snapshot_count; ++replay) {
    std::ostringstream actual;
    Runner restored(input.firmware, input.helper, phase, secure, actual, &saved.target);
    restored.restore(saved);
    restored.until_fault();
    require(
        actual.str() == suffix && restored.endpoint() == run.endpoint() &&
            restored.pair.target.memory().flash_image() == run.pair.target.memory().flash_image() &&
            restored.pair.target.memory().flash_state() == run.pair.target.memory().flash_state() &&
            restored.pair.target.cpu_state() == run.pair.target.cpu_state() &&
            restored.pair.target.bdc_state() == run.pair.target.bdc_state() &&
            restored.pair.target.wire_state() == run.pair.target.wire_state() &&
            restored.pair.target.run_state() == run.pair.target.run_state(),
        "Fresh host/target CPU/RAM/BDC/wire/clock owners reproduce every complete native "
        "suffix and failure");
    trace << "REPLAY index=" << replay << " bytes=" << actual.str().size() << " identical=1\n";
  }
  trace << "CASE_END phase=" << phase << " secure=" << secure << " host-steps=" << run.pair.steps
        << " target-retired=" << run.pair.target.run_state().retired_instructions
        << " target-executing=" << run.pair.target.run_state().executing_periods
        << " commands=" << run.pair.target.bdc_state().serviced
        << " rx=" << run.pair.target.wire_state().received_bytes
        << " tx=" << run.pair.target.wire_state().transmitted_bytes << '\n';
}

int execute(const int count, char **arguments) {
  require(count == 5, "Expected original MCU firmware, complete helper, records and private trace");
  const auto firmware = Firmware::load(arguments[1]);
  const auto helper = blofeld::test::keyboard_receiver::bytes(
      arguments[2], blofeld::test::keyboard_board_transport::helper_bytes);
  const auto records =
      blofeld::test::keyboard_receiver::bytes(arguments[3], record_count * record_bytes);
  const Inputs input{.firmware = firmware, .helper = helper, .records = records};
  std::ofstream trace(arguments[4]);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  run_case(input, 0, false, trace);
  run_case(input, 4, false, trace);
  run_case(input, 0, true, trace);
  trace << "END cases=3 replays=6 RAM-uploads=3 launched=3 NVM-transfers=0\n";
  std::cout
      << "Original HC12 upload/GO reaches production HCS08 reset-known strict frontiers through "
         "GPIO.\n";
  return 0;
}
} // namespace

int main(const int count, char **arguments) {
  try {
    return execute(count, arguments);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
