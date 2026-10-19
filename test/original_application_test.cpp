// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "application_midi_fixture.hpp"
#include "application_update_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "lcd_fixture.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
void run(const std::filesystem::path &reference, std::ostream &trace, std::ostream &updates) {
  const auto firmware = blofeld::Firmware::load(reference);
  for (const bool high : {false, true}) {
    trace << "case pm3=" << (high ? 1 : 0) << '\n';
    blofeld::McuBus bus(firmware.mcu);
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    blofeld::test::LcdFixture display(bus);
    static_cast<void>(blofeld::test::start_application(bus, cpu, display, trace, high));
    trace << "native-sci baud-high=" << blofeld::hex(bus.read(blofeld::hc12::sci_baud_high), 2)
          << " baud-low=" << blofeld::hex(bus.read(blofeld::hc12::sci_baud_low), 2)
          << " control1=" << blofeld::hex(bus.read(blofeld::hc12::sci_control1), 2)
          << " control2=" << blofeld::hex(bus.read(blofeld::hc12::sci_control2), 2)
          << " modrr=" << blofeld::hex(bus.read(blofeld::hc12::modrr), 2) << '\n';
    if (!high) { blofeld::test::verify_application_midi(bus, cpu, trace); }
    updates << "case pm3=" << (high ? 1 : 0) << '\n';
    blofeld::test::verify_application_update(bus, cpu, firmware, updates);
    bus.set_observer({});
  }
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-application-test firmware.zip private.trace");
    }
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    auto update_path = std::filesystem::path(arguments[2]);
    update_path += ".update";
    std::ofstream updates(update_path);
    updates.exceptions(std::ios::failbit | std::ios::badbit);
    run(arguments[1], trace, updates);
    std::cout << "Original application: native reset/RTI/display/main WAI, missing-PM3 retry, "
                 "both controlled input levels, 264 SCI receives, two original update requests "
                 "through first-helper return/drain and main-helper flash initialization, and "
                 "unchanged MCU NVM verified. "
                 "Conditional "
                 "GPIO/USB/ready/clock fixture; idle FSTAT CBEIF write still unqualified. "
                 "No measured board, full update or DSP boot.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
