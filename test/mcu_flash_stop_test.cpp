// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "flash_stop_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::flash_stop;
enum class Phase : std::uint8_t { launched, active, queued };

struct Fixture {
  Fixture(const unsigned opcode, const Phase phase) : operation(opcode), point(phase) {
    configure(bus);
    erase_sector(bus, sector);
    erase_sector(bus, successor);
    before = bus.power_off_flash_image();
    launch(bus, {.opcode = operation});
    if (point != Phase::launched) { bus.advance_flash(FlashMilestone::transfer_buffer); }
    if (point == Phase::queued) {
      launch(bus, {.opcode = program, .address = successor, .value = next_data});
    }
  }

  void check() const {
    const auto actual = bus.power_off_flash_image();
    unsigned count = array_bytes;
    unsigned start = 0;
    if (operation == program) {
      count = 2;
      start = target - base;
    }
    if (operation == erase) {
      count = sector_bytes;
      start = sector - base;
    }
    for (unsigned offset = 0; offset < array_bytes; ++offset) {
      const bool lost = point != Phase::launched && operation != verify && offset >= start &&
                        offset - start < count;
      require(
          actual.known[offset] == (before.known[offset] && !lost) &&
              actual.bytes[offset] == (lost ? 0 : before.bytes[offset]) &&
              actual.fresh[offset / 2] == (before.fresh[offset / 2] && !lost),
          "STOP loses only the active program/sector/block extent, never an unexecuted successor");
    }
  }

  McuBus bus{image()};
  McuFlashImage before;
  unsigned operation;
  Phase point;
};

void test_all_operations_and_phases() {
  for (const unsigned opcode : {program, erase, mass, verify}) {
    for (const Phase phase : {Phase::launched, Phase::active, Phase::queued}) {
      Fixture fixture(opcode, phase);
      enter(fixture.bus);
      fixture.check();
      require(
          rejected([&] { static_cast<void>(fixture.bus.read(status)); }).contains("STOP is held") &&
              rejected([&] {
                fixture.bus.advance_flash(FlashMilestone::complete_command);
              }).contains("STOP is held"),
          "Held STOP cannot invent a readable CBEIF or complete the aborted command");
      auto branch = fixture.bus.clone();
      resume(*branch);
      require(branch->read(status) == (idle | access_error),
              "Nominal STOP wake sets CBEIF/CCIF/ACCERR for every defined active-command context");
      require(
          rejected([&] { static_cast<void>(fixture.bus.read(status)); }).contains("STOP is held"),
          "A copied peripheral retains STOP independently from the resumed branch");
      resume(fixture.bus);
      require(fixture.bus.read(status) == (idle | access_error) &&
                  rejected([&] {
                    static_cast<void>(fixture.bus.read(command));
                  }).contains("Unknown MCU flash FCMD"),
              "Wake keeps the documented abort status and no invented command-buffer readback");
      fixture.bus.write(status, access_error);
      require(fixture.bus.read(status) == idle,
              "Software can clear ACCERR after wake without reviving aborted work");
      static_cast<void>(
          rejected([&] { fixture.bus.advance_flash(FlashMilestone::transfer_buffer); }));
      static_cast<void>(
          rejected([&] { fixture.bus.advance_flash(FlashMilestone::complete_command); }));
      fixture.check();
      same_image(branch->power_off_flash_image(), fixture.bus.power_off_flash_image());
    }
  }
}

void test_held_irq_knowledge() {
  constexpr unsigned ccif_enable = 0x40;
  constexpr unsigned cbeif_enable = 0x80;
  for (const unsigned enables : {0U, ccif_enable, cbeif_enable, idle}) {
    Fixture fixture(program, Phase::active);
    fixture.bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    fixture.bus.observe_interrupt_pads();
    fixture.bus.write(configuration, static_cast<std::uint8_t>(enables));
    enter(fixture.bus);
    if (enables == cbeif_enable) {
      require(rejected([&] {
                static_cast<void>(fixture.bus.interrupt_request(0));
              }).contains("CBEIF interrupt"),
              "CBEIF-only interrupt knowledge is not invented before STOP exit");
    } else {
      require(fixture.bus.interrupt_request(0).has_value() == (enables != 0),
              "Documented abort CCIF asserts its enabled IRQ even with unknown CBEIF");
    }
    resume(fixture.bus);
    require(fixture.bus.interrupt_request(0).has_value() == (enables != 0),
            "Wake CBEIF and CCIF have known enabled/disabled interrupt outcomes");
  }
}

void test_qualification_preserves_command() {
  Fixture fixture(program, Phase::active);
  require(rejected([&] {
            fixture.bus.observe_flash_stop(FlashStopTransition::enter);
          }).contains("explicit nominal"),
          "The unqualified default cannot abort a command or select a silicon mask");
  static_cast<void>(rejected([&] { resume(fixture.bus); }));
  fixture.bus.advance_flash(FlashMilestone::complete_command);
  require(fixture.bus.read_memory({.address = target, .size = 2}) == word_data &&
              fixture.bus.read(status) == idle,
          "Rejected observations leave the original active program fully completable");
  static_cast<void>(rejected([&] { enter(fixture.bus); }));
  require(fixture.bus.read(status) == idle, "Idle STOP remains outside the active-abort contract");
}

void test_partial_successor() {
  for (const bool selected : {false, true}) {
    Fixture fixture(program, Phase::active);
    fixture.bus.write_memory({.address = successor, .size = 2, .value = next_data});
    if (selected) { fixture.bus.write(command, program); }
    require(rejected([&] { enter(fixture.bus); }).contains("partial successor"),
            "The module reference does not guess STOP during an unlaunched command sequence");
    fixture.bus.advance_flash(FlashMilestone::complete_command);
    if (!selected) { fixture.bus.write(command, program); }
    fixture.bus.write(status, launch_bit);
    finish(fixture.bus);
    require(fixture.bus.read_memory({.address = successor, .size = 2}) == next_data,
            "An unsupported STOP observation leaves both valid command sequences intact");
  }
}

void test_native_cached_fetch() {
  constexpr unsigned load_immediate = 0x86;
  constexpr unsigned first_result = 0x0100;
  constexpr unsigned interrupts_masked_stop_enabled = 0x50;
  auto nvm = image();
  nvm.bytes[sector - base] = load_immediate;
  nvm.bytes[sector - base + 1] = 1;
  McuBus bus(nvm);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state({.pc = sector, .ccr = interrupts_masked_stop_enabled});
  cpu.step();
  require(cpu.state().d == first_result,
          "Own native LDAA immediate is cached before the erase abort");
  configure(bus);
  launch(bus, {.opcode = erase, .address = sector});
  bus.advance_flash(FlashMilestone::transfer_buffer);
  enter(bus);
  cpu.set_state({.pc = sector, .ccr = interrupts_masked_stop_enabled});
  require(rejected([&] { cpu.step(); }).contains("STOP is held"),
          "A cached opcode cannot execute through the held flash STOP boundary");
  resume(bus);
  cpu.set_state({.pc = sector, .ccr = interrupts_masked_stop_enabled});
  require(rejected([&] {
            cpu.step();
          }).contains("Uninitialized or unmapped MCU instruction fetch at 0x8400"),
          "After wake the native code cache cannot replace the lost opcode with old bytes");
}

void test_recovery_and_context() {
  Fixture fixture(program, Phase::active);
  fixture.bus.begin_instruction({.pc = word_data, .ccr = idle});
  fixture.bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  fixture.bus.observe_interrupt_pads();
  fixture.bus.write(configuration, launch_bit);
  enter(fixture.bus);
  const auto interrupt_fault =
      rejected([&] { static_cast<void>(fixture.bus.interrupt_request(0)); });
  require(rejected([&] { enter(fixture.bus); }).contains("pc=0x1234") &&
              rejected([&] { static_cast<void>(fixture.bus.read(status)); }).contains("0x0105") &&
              interrupt_fault.contains("CBEIF interrupt") &&
              interrupt_fault.contains("vector=0xffb8 pc=0x1234 ccr=0x00"),
          "Repeated STOP, held register and enabled IRQ observations retain visible context");
  resume(fixture.bus);
  require(fixture.bus.interrupt_request(0).has_value(),
          "CCIF can request the configured interrupt after wake");
  fixture.bus.write(configuration, 0);
  fixture.bus.reset();
  fixture.check();
  require(fixture.bus.read(command) == 0 && fixture.bus.read(status) == idle,
          "Reset restores volatile command knowledge but never repairs lost flash contents");
  configure(fixture.bus);
  erase_sector(fixture.bus, sector);
  launch(fixture.bus);
  finish(fixture.bus);
  require(fixture.bus.read_memory({.address = target, .size = 2}) == word_data,
          "Only an actual erase/program sequence recovers the unknown word");
  Fixture mass_abort(mass, Phase::active);
  enter(mass_abort.bus);
  resume(mass_abort.bus);
  require(rejected([&] { mass_abort.bus.reset(); }).contains("Unknown physical MCU flash byte"),
          "Aborted mass erase cannot manufacture option/security knowledge for reset");
}
}

int main() {
  try {
    test_all_operations_and_phases();
    test_qualification_preserves_command();
    test_partial_successor();
    test_recovery_and_context();
    test_held_irq_knowledge();
    test_native_cached_fetch();
    std::cout << "Nominal active flash STOP abort, pending cancellation, NVM knowledge and "
                 "recovery verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
