// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard_bdc_wire_fixture.hpp"
#include "keyboard_receiver_fixture.hpp"
#include "keyboard_target_fixture.hpp"

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
using namespace blofeld::test::keyboard_target;
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
    Runner restored(input.firmware, input.helper, phase, secure, actual, &saved.memory);
    restored.restore(saved);
    restored.until_fault();
    require(actual.str() == suffix && restored.endpoint() == run.endpoint() &&
                restored.pair.target.memory.bytes == run.pair.target.memory.bytes &&
                restored.pair.target.memory.known == run.pair.target.memory.known &&
                restored.pair.target.memory.flash.image() == run.pair.target.memory.flash.image() &&
                restored.pair.target.memory.flash.state() == run.pair.target.memory.flash.state() &&
                restored.pair.bdc.state() == run.pair.bdc.state() &&
                restored.pair.wire.state() == run.pair.wire.state() &&
                restored.clock.state() == run.clock.state(),
            "Fresh host/target CPU/RAM/BDC/wire/clock owners reproduce every complete native "
            "suffix and failure");
    trace << "REPLAY index=" << replay << " bytes=" << actual.str().size() << " identical=1\n";
  }
  trace << "CASE_END phase=" << phase << " secure=" << secure << " host-steps=" << run.pair.steps
        << " target-retired=" << run.clock.state().retired_instructions
        << " target-executing=" << run.clock.state().executing_periods
        << " commands=" << run.pair.bdc.state().serviced
        << " rx=" << run.pair.wire.state().received_bytes
        << " tx=" << run.pair.wire.state().transmitted_bytes << '\n';
}

int execute(const int count, char **arguments) {
  require(count == 5, "Expected original MCU firmware, complete helper, records and private trace");
  const auto firmware = Firmware::load(arguments[1]);
  const auto helper = blofeld::test::keyboard_receiver::bytes(
      arguments[2], blofeld::test::keyboard_bdc_wire::helper_bytes);
  const auto records =
      blofeld::test::hcs08_programmer::read_input(arguments[3], record_count * record_bytes);
  const Inputs input{.firmware = firmware, .helper = helper, .records = records};
  std::ofstream trace(arguments[4]);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  run_case(input, 0, false, trace);
  run_case(input, 4, false, trace);
  run_case(input, 0, true, trace);
  trace << "END cases=3 replays=6 RAM-uploads=3 launched=3 NVM-transfers=0\n";
  std::cout << "Original HC12 RAM upload and PC/SP/GO reach native HCS08 strict frontiers through "
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
