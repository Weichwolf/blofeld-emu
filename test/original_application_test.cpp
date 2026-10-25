// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "application_midi_fixture.hpp"
#include "application_update_fixture.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "storage/flash_image.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

namespace {
void verify_checkpoint_replay(blofeld::McuMachine &machine,
                              std::unique_ptr<blofeld::test::LcdFixture> &display,
                              std::ostream &trace) {
  const auto checkpoint = machine.checkpoint();
  const blofeld::test::LcdFixture frozen_display(machine.bus(), *display);
  std::ostringstream original;
  blofeld::test::verify_application_midi(machine.bus(), machine.cpu(), original);
  const auto before = machine.cpu().state();
  const auto counts = display->counts();
  machine.restore(checkpoint);
  display = std::make_unique<blofeld::test::LcdFixture>(machine.bus(), frozen_display);
  machine.bus().set_observer(
      [&display](const blofeld::BusEvent &event) { display->observe(event); });
  std::ostringstream replay;
  blofeld::test::verify_application_midi(machine.bus(), machine.cpu(), replay);
  const auto after = machine.cpu().state();
  const auto replay_counts = display->counts();
  if (original.str() != replay.str() || !machine.cpu().waiting() || before.pc != after.pc ||
      before.d != after.d || before.x != after.x || before.y != after.y || before.sp != after.sp ||
      before.ccr != after.ccr || counts.commands != replay_counts.commands ||
      counts.writes != replay_counts.writes || counts.reads != replay_counts.reads) {
    throw blofeld::Fault("Original cold-WAI checkpoint replay differs from native SCI execution");
  }
  trace << original.str();
}

void run(const std::filesystem::path &reference, std::ostream &trace, std::ostream &updates) {
  const auto firmware = blofeld::Firmware::load(reference);
  for (const bool high : {false, true}) {
    trace << "case pm3=" << (high ? 1 : 0) << '\n';
    blofeld::McuMachine machine(blofeld::McuFlashImage::from_known(firmware.mcu),
                                BLOFELD_DEFAULT_SLA);
    auto display = std::make_unique<blofeld::test::LcdFixture>(machine.bus());
    static_cast<void>(blofeld::test::start_application(machine, display, trace, high));
    auto &bus = machine.bus();
    trace << "native-sci baud-high=" << blofeld::hex(bus.read(blofeld::hc12::sci_baud_high), 2)
          << " baud-low=" << blofeld::hex(bus.read(blofeld::hc12::sci_baud_low), 2)
          << " control1=" << blofeld::hex(bus.read(blofeld::hc12::sci_control1), 2)
          << " control2=" << blofeld::hex(bus.read(blofeld::hc12::sci_control2), 2)
          << " modrr=" << blofeld::hex(bus.read(blofeld::hc12::modrr), 2) << '\n';
    if (!high) { verify_checkpoint_replay(machine, display, trace); }
    updates << "case pm3=" << (high ? 1 : 0) << '\n';
    blofeld::test::verify_application_update(machine, display, firmware, updates);
    machine.bus().set_observer({});
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
    std::cout
        << "Original application: native reset/RTI/display/main WAI, coordinated PM3 recovery, "
           "both controlled input levels, 264 SCI receives plus cold-WAI checkpoint replay, "
           "two original update requests "
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
