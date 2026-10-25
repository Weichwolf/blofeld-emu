// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard_receiver_fixture.hpp"
#include "keyboard_reset_sync_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_reset_sync;
constexpr unsigned nominal_hz = 4000000, byte_mask = 255, word_mask = 65535;
constexpr unsigned background_command = 0x90, low_delay = 0x1600, high_delay = 0x1601;

struct Case {
  Reference reference;
  unsigned result;
};

constexpr std::array cases{
    Case{.reference =
             {.bdc_hz = nominal_hz, .phase_ticks = 1, .reset_pullup = true, .connected = true},
         .result = 9},
    Case{.reference =
             {.bdc_hz = nominal_hz, .phase_ticks = 3, .reset_pullup = true, .connected = true},
         .result = 9},
    Case{
        .reference = {.bdc_hz = 3000000, .phase_ticks = 1, .reset_pullup = true, .connected = true},
        .result = 0},
    Case{
        .reference = {.bdc_hz = 6000000, .phase_ticks = 1, .reset_pullup = true, .connected = true},
        .result = 0},
    Case{.reference =
             {.bdc_hz = nominal_hz, .phase_ticks = 1, .reset_pullup = false, .connected = true},
         .result = 0},
    Case{.reference =
             {.bdc_hz = nominal_hz, .phase_ticks = 1, .reset_pullup = true, .connected = false},
         .result = 0}};

void retain(const Fixture &fixture) {
  const auto nvm = fixture.host.bus().power_off_flash_image();
  require(std::ranges::equal(nvm.bytes, fixture.firmware.mcu) && nvm.known.all() &&
              nvm.fresh.none(),
          "Original reset/SYNC preserves every MCU NVM byte and its knowledge/history");
  for (unsigned n = 0; n < fixture.helper.size(); ++n) {
    require(fixture.host.bus().peek(static_cast<std::uint16_t>(helper_base + n)) ==
                fixture.helper[n],
            "All1018 original helper bytes remain unchanged");
  }
  if (fixture.target) {
    require(fixture.target->memory.retired == 0 &&
                fixture.target->cpu.state() ==
                    Hcs08State{.pc = hw::algorithm, .hx = hw::seed, .sp = hw::stack},
            "The declared target context stays held; no target reset/application execution is "
            "invented");
  }
}

void execute(const Firmware &firmware,
             const std::vector<std::uint8_t> &helper,
             const unsigned index,
             const std::string &path) {
  std::ofstream trace(path + "." + std::to_string(index));
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  const auto test = cases.at(index);
  trace << "CASE index=" << index << " rate=" << test.reference.bdc_hz
        << " phase=" << test.reference.phase_ticks << " pullup=" << test.reference.reset_pullup
        << " connected=" << test.reference.connected << '\n';
  Fixture fixture(firmware, helper, test.reference, trace);
  if (!test.reference.reset_pullup) {
    const auto error = blofeld::test::hcs08::rejected([&] { fixture.run(); });
    require(error.contains("Unknown MCU GPIO input at 0x0008 mask=0x80") &&
                error.contains("pc=0x13b0") && fixture.asserted && !fixture.released &&
                !fixture.wire && fixture.host.timing_faulted(),
            "Unobserved reset release fails at the real original PE7 read");
    trace << "FAULT pc=" << fixture.host.cpu().state().pc << " steps=" << fixture.steps
          << " error=" << error << '\n';
  } else {
    fixture.run();
    require((fixture.host.cpu().state().d & byte_mask) == test.result &&
                fixture.host.cpu().state().sp == stack && fixture.asserted && fixture.released,
            "Original success/rejection returns through its actual native stack frame");
    if (test.reference.connected) {
      require(fixture.wire && fixture.bdc && fixture.wire->state().syncs == 2 &&
                  fixture.bdc->state().serviced == (test.result == 9 ? 1U : 0U) &&
                  fixture.wire->state().received_bytes == (test.result == 9 ? 1U : 0U) &&
                  (test.result == 0 || fixture.bdc->state().frame[0] == background_command) &&
                  fixture.difference == ((fixture.high_read - fixture.low_read) & word_mask),
              "Native polled timestamp difference and actual SYNC/BACKGROUND outcomes agree");
    }
    if (test.result == 9) {
      require(fixture.host.bus().read(low_delay) == 9 && fixture.host.bus().read(high_delay) == 12,
              "Original instructions, not fixture presets, install the9/12 byte delays");
    }
  }
  retain(fixture);
  trace << "CHECK helper=1 nvm=1 target-instructions=0\n";
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected original firmware, helper and trace prefix");
    const auto firmware = Firmware::load(arguments[1]);
    const auto helper = blofeld::test::keyboard_receiver::bytes(arguments[2], helper_bytes);
    for (unsigned index = 0; index < cases.size(); ++index) {
      execute(firmware, helper, index, arguments[3]);
    }
    std::cout << "Original HC12 reset/SYNC reference clocks, GPIO and polled timer verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
