// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard_bdc_wire_fixture.hpp"
#include "keyboard_receiver_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_bdc_wire;
constexpr unsigned byte_values = 256, phase_cases = 8;
constexpr std::array<unsigned, 4> corners{0, 255, 0x55, 0xaa};

constexpr unsigned active_status = 0xc8;

void guards(const Fixture &fixture) {
  const auto nvm = fixture.host.bus().power_off_flash_image();
  require(std::ranges::equal(nvm.bytes, fixture.firmware.mcu) && nvm.known.all() &&
              nvm.fresh.none(),
          "All original MCU NVM bytes/knowledge/history remain unchanged by BDC byte helpers");
  for (unsigned n = 0; n < fixture.helper.size(); ++n) {
    require(fixture.host.bus().peek(static_cast<std::uint16_t>(helper_base + n)) ==
                fixture.helper[n],
            "Every original HC12 helper byte remains unchanged");
  }
  require(fixture.target.cpu.state() == Hcs08State{.pc = blofeld::test::hcs08_wire::algorithm,
                                                   .hx = blofeld::test::hcs08_wire::seed,
                                                   .sp = blofeld::test::hcs08_wire::stack} &&
              fixture.target.memory.retired == 0 && fixture.bdc.status() == active_status,
          "Native BDC memory commands leave the actual held HCS08 registers/execution untouched");
}

int execute(const int count, char **arguments) {
  require(count == 4,
          "Expected official main firmware, complete Keyboard helper and private trace");
  const auto firmware = Firmware::load(arguments[1]);
  const auto helper = blofeld::test::keyboard_receiver::bytes(arguments[2], helper_bytes);
  std::ofstream trace(arguments[3]);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  unsigned total = 0;
  std::uint64_t instructions = 0;
  std::uint64_t writes = 0;
  for (unsigned phase = 0; phase < phase_cases; ++phase) {
    Fixture fixture(firmware, helper, phase, trace);
    trace << "PHASE index=" << phase << " ratio=" << reference_ratio
          << " offset=" << ((phase * 2U) + 1U) << '\n';
    unsigned cases = 0;
    if (phase == 0 || phase == 4) {
      for (unsigned value = 0; value < byte_values; ++value) {
        fixture.pair(value);
        ++cases;
      }
    } else {
      for (const auto value : corners) {
        fixture.pair(value);
        ++cases;
      }
    }
    guards(fixture);
    require(fixture.bdc.state().serviced == static_cast<std::uint64_t>(cases) * 2 &&
                fixture.wire.state().received_bytes == static_cast<std::uint64_t>(cases) * 7 &&
                fixture.wire.state().transmitted_bytes == cases,
            "All native HC12 transfers reach exactly the intended BDC target commands and replies");
    trace << "PHASE_END index=" << phase << " cases=" << cases
          << " commands=" << static_cast<std::uint64_t>(cases) * 2
          << " rx=" << fixture.wire.state().received_bytes
          << " tx=" << fixture.wire.state().transmitted_bytes << " steps=" << fixture.steps
          << " cycles=" << fixture.host.run_clock().bus_cycles()
          << " pin-writes=" << fixture.pin_writes << '\n';
    total += cases;
    instructions += fixture.steps;
    writes += fixture.pin_writes;
  }
  trace << "END phases=8 cases=" << total << " commands=" << total * 2 << " rx=" << total * 7
        << " tx=" << total << " steps=" << instructions << " pin-writes=" << writes << '\n';
  std::cout << "Original HC12 BDC GPIO byte helpers and actual target memory/replies verified.\n";
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
