// SPDX-License-Identifier: GPL-3.0-only
#include "flash_backdoor_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test::flash_key {
namespace {
constexpr unsigned initial_x = 0x1234;
constexpr unsigned initial_y = 0x5678;
constexpr unsigned initial_flags = 0xd0;
constexpr unsigned final_flags = 0xd4;
constexpr unsigned final_good_d = 0x0071;
constexpr unsigned final_bad_d = 0x0070;
constexpr unsigned partial_pc = 0x1811;
constexpr unsigned changed_operand = 0x1819;
constexpr unsigned changed_low_byte = 0x70;
constexpr unsigned ram_entry = 0x1800;
constexpr unsigned stack = 0x1ff0;
constexpr unsigned total_instructions = 12;
constexpr unsigned partial_instructions = 6;
// Own HC12 LDAA/STAA, four LDD/STD pairs, LDAA/STAA. No manufacturer bytes.
constexpr std::array<std::uint8_t, 34> program{0x86, 0x20, 0x7a, 1,    3,    0xcc, 0x13, 0x57, 0x7c,
                                               0xff, 0,    0xcc, 0x24, 0x68, 0x7c, 0xff, 2,    0xcc,
                                               0x5a, 0x3c, 0x7c, 0xff, 4,    0xcc, 0x8e, 0x71, 0x7c,
                                               0xff, 6,    0x86, 0,    0x7a, 1,    3};

void initialize(McuMachine &machine) {
  for (unsigned i = 0; i < program.size(); ++i) {
    machine.bus().write(static_cast<std::uint16_t>(ram_entry + i), program.at(i));
  }
  machine.cpu().set_state(
      {.pc = ram_entry, .x = initial_x, .y = initial_y, .sp = stack, .ccr = initial_flags});
}

void advance(McuMachine &machine, const unsigned count) {
  for (unsigned i = 0; i < count; ++i) {
    const auto step = machine.cpu().step();
    require(step.instruction_executed && !step.interrupt_vector,
            "Each key instruction must execute through actual HC12 semantics without an IRQ");
  }
}

void endpoint(McuMachine &machine, const bool matched) {
  const auto state = machine.cpu().state();
  require(state.pc == ram_entry + program.size() &&
              state.d == (matched ? final_good_d : final_bad_d) && state.x == initial_x &&
              state.y == initial_y && state.sp == stack && state.ccr == final_flags &&
              machine.bus().read(security) == (matched ? unsecured : secured) &&
              machine.bus().read(status) == idle_status && machine.bus().read(protection) == 0,
          "Independent native LDD/STD/LDAA flags/registers/PC accompany only the valid unlock");
}

void test_locked_owner_capture() {
  const auto before = image();
  McuMachine machine(before, BLOFELD_DEFAULT_SLA);
  initialize(machine);
  machine.bus().write(changed_operand, changed_low_byte);
  constexpr unsigned through_last_store = 10;
  constexpr unsigned locked_pc = 0x181d;
  constexpr unsigned locked_d = 0x8e70;
  constexpr unsigned locked_flags = 0xd8;
  advance(machine, through_last_store);
  const auto state = machine.cpu().state();
  require(state.pc == locked_pc && state.d == locked_d && state.ccr == locked_flags &&
              machine.bus().read(control) == key_access && machine.bus().read(security) == secured,
          "The actual fourth native store captures the independently expected mismatch endpoint");
  const auto locked = machine.checkpoint();
  machine.bus().reset();
  machine.bus().write(control, key_access);
  feed(machine.bus());
  machine.bus().write(control, 0);
  require(machine.bus().read(security) == unsecured,
          "Reset independently releases the native lock");
  machine.restore(locked);
  require(machine.cpu().state().pc == locked_pc && machine.bus().read(control) == key_access,
          "Joint restore recovers the actual locked native prefix before KEYACC close");
  constexpr unsigned fourth = 3;
  put(machine.bus(), fourth, keys.back());
  machine.bus().write(control, 0);
  require(
      machine.bus().read(security) == secured,
      "A corrected fourth word after locked restore cannot complete the saved three-word prefix");
  unchanged(machine.bus(), before);
}

void test_successful_owner_and_cold_projection() {
  const auto before = image();
  McuMachine machine(before, BLOFELD_DEFAULT_SLA);
  initialize(machine);
  advance(machine, total_instructions);
  endpoint(machine, true);
  const auto unlocked = machine.checkpoint();
  machine.bus().power_cycle();
  require(machine.bus().read(security) == secured && machine.bus().read(control) == 0,
          "A power cycle independently removes the successful volatile unlock");
  machine.restore(unlocked);
  endpoint(machine, true);
  unchanged(machine.bus(), before);
  const auto encoded = encode_flash_image(machine.bus().power_off_flash_image());
  McuBus cold(decode_flash_image(encoded));
  require(cold.read(security) == secured && cold.read(control) == 0 && cold.read(protection) == 0,
          "A cold NVM reconstruction cannot import volatile unlock or key-machine state");
  unchanged(cold, before);
}

void test_previously_executed_flash_fetch() {
  const auto before = image();
  McuMachine machine(before, BLOFELD_DEFAULT_SLA);
  machine.cpu().set_state({.pc = flash_base, .sp = stack, .ccr = initial_flags});
  advance(machine, 1);
  require(machine.cpu().state().pc == flash_base + 1, "An own Flash NOP executes before KEYACC");
  const auto version = machine.bus().code_revision();
  initialize(machine);
  constexpr unsigned enable_instructions = 2;
  advance(machine, enable_instructions);
  require(machine.bus().code_revision() == version && machine.bus().read(control) == key_access,
          "Native KEYACC entry changes read validity without changing the previously decoded code");
  machine.cpu().set_state({.pc = flash_base, .sp = stack, .ccr = initial_flags});
  reject([&] { static_cast<void>(machine.cpu().step()); }, "KEYACC is set");
  require(machine.cpu().state().pc == flash_base,
          "Previously executed Flash code cannot bypass the live KEYACC fetch guard");
  unchanged(machine.bus(), before);
}
}

void test_native_keys_and_partial_owner_restore() {
  test_locked_owner_capture();
  test_successful_owner_and_cold_projection();
  test_previously_executed_flash_fetch();
  const auto before = image();
  McuMachine machine(before, BLOFELD_DEFAULT_SLA);
  initialize(machine);
  advance(machine, partial_instructions);
  require(machine.cpu().state().pc == partial_pc && machine.bus().read(control) == key_access &&
              machine.bus().read(security) == secured,
          "The real mid-sequence snapshot has completed exactly two native key stores");
  const auto middle = machine.checkpoint();
  advance(machine, total_instructions - partial_instructions);
  endpoint(machine, true);
  unchanged(machine.bus(), before);
  machine.restore(middle);
  // Change only one unexecuted own instruction operand; copied native state stays original.
  machine.bus().write(changed_operand, changed_low_byte);
  advance(machine, total_instructions - partial_instructions);
  endpoint(machine, false);
  unchanged(machine.bus(), before);
  machine.bus().write(control, key_access);
  feed(machine.bus());
  machine.bus().write(control, 0);
  require(machine.bus().read(security) == secured, "The native mismatch lock survives another try");
  machine.restore(middle);
  advance(machine, total_instructions - partial_instructions);
  endpoint(machine, true);
  unchanged(machine.bus(), before);
  machine.bus().reset();
  require(machine.bus().read(security) == secured,
          "Native success leaves reset security unchanged");
  machine.restore(middle);
  machine.cpu().set_state({.pc = flash_base, .ccr = initial_flags});
  reject([&] { static_cast<void>(machine.cpu().step()); }, "KEYACC is set");
  require(machine.cpu().state().pc == flash_base,
          "A flash fetch cannot advance past invalid key-mode data");
}
}
