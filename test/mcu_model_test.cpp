// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_memory.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
// Independent expectations: 9S12UF32DGV1 table 1-1 and section 18,
// S12MMCV4 table 4-1, S12SMRAM3P5K2EV1 section 4, S12UF32PIMV2 section 3.
constexpr unsigned upper_flash = 0xc000;
constexpr unsigned qram_at_2000 = 0x21;
constexpr unsigned initrm = 0x10;
constexpr unsigned initrg = 0x11;
constexpr unsigned initee = 0x12;
constexpr unsigned misc = 0x13;
constexpr unsigned reset_ram = 0x1200;
constexpr unsigned relocated_ram = 0x2a00;
constexpr unsigned reset_qram = 0x0400;
constexpr unsigned relocated_qram = 0x2400;
constexpr unsigned ram_end = 0x1fff;
constexpr unsigned relocated_ram_end = 0x37ff;
constexpr unsigned port_e = 8;
constexpr unsigned direction_e = 9;
constexpr unsigned assignment_e = 10;
constexpr unsigned q_data = 0x260;
constexpr unsigned q_input = 0x261;
constexpr unsigned q_direction = 0x262;
constexpr unsigned q_pull_enable = 0x264;
constexpr unsigned q_pull_select = 0x265;
constexpr unsigned routing = 0x257;
constexpr unsigned all_pins = 0xff;
constexpr unsigned partial_pins = 0xfb;
constexpr unsigned input_high = 8;
constexpr unsigned pattern = 0x55;
constexpr unsigned inverse_pattern = 0xaa;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

std::array<std::uint8_t, 32768> flash_fixture() {
  std::array<std::uint8_t, 32768> flash{};
  flash[0] = pattern;
  flash[0x4000] = inverse_pattern;
  return flash;
}

void test_reset_map_and_unknown_ram() {
  const auto flash = flash_fixture();
  blofeld::McuMemory memory(flash);
  require(memory.read(0x8000) == pattern && memory.read(0x4000) == pattern,
          "Lower flash page and its fixed alias");
  require(memory.read(upper_flash) == inverse_pattern, "Upper fixed flash page");
  require(memory.locate(0).space == blofeld::McuSpace::registers, "Register block priority");
  reject([&memory] { static_cast<void>(memory.read(reset_ram)); }, "Cold SMRAM is not initialized");
  reject([&memory] { static_cast<void>(memory.read(reset_qram)); }, "Cold QRAM is not initialized");
  reject([&memory] { memory.check_execute(reset_ram); }, "Cannot execute unknown RAM bytes");
  constexpr unsigned hole = reset_ram - 1;
  reject([&memory] { static_cast<void>(memory.read(hole)); },
         "Reserved RAM holes are unsupported reads");
  reject([&memory] { memory.write({.address = hole, .value = pattern}); },
         "Reserved holes do not become RAM on writes");
  reject([&memory] { memory.check_execute(hole); }, "Reserved holes are not executable");
  memory.write({.address = reset_ram, .value = pattern});
  memory.write({.address = ram_end, .value = inverse_pattern});
  require(memory.read(reset_ram) == pattern && memory.read(ram_end) == inverse_pattern,
          "Both ends of 3584-byte reset SMRAM");
  memory.check_execute(reset_ram);
  reject([&memory] { static_cast<void>(memory.read(ram_end + 1)); },
         "No speculative RAM after 1fff");
}

void test_ram_remap_and_reset() {
  const auto flash = flash_fixture();
  blofeld::McuMemory memory(flash);
  memory.write({.address = reset_ram, .value = pattern});
  memory.write({.address = ram_end, .value = inverse_pattern});
  constexpr unsigned new_initrm = 0x20;
  constexpr unsigned initial_initrm = 0x09;
  memory.write_register({.address = initrm, .value = new_initrm});
  require(memory.read_register(initrm) == initial_initrm && memory.read(reset_ram) == pattern,
          "Mapping remains active through the writing instruction");
  require(memory.retire_instruction(), "Mapping update is committed at instruction retirement");
  require(memory.read_register(initrm) == new_initrm, "New INITRM readback");
  require(memory.read(relocated_ram) == pattern &&
              memory.read(relocated_ram_end) == inverse_pattern,
          "Remapping relocates the same physical RAM contents");
  reject([&memory] { static_cast<void>(memory.read(reset_ram)); }, "Old RAM location is unmapped");
  memory.write_register({.address = initrm, .value = initial_initrm});
  require(!memory.retire_instruction() && memory.read_register(initrm) == new_initrm,
          "INITRM is write-once in normal mode");
  memory.reset();
  require(memory.read(reset_ram) == pattern && memory.read(ram_end) == inverse_pattern,
          "External reset restores the map and retains RAM");
}

void test_qram_relocation_and_priority() {
  const auto flash = flash_fixture();
  blofeld::McuMemory memory(flash);
  memory.write({.address = reset_qram, .value = pattern});
  memory.write_register({.address = initee, .value = qram_at_2000});
  memory.retire_instruction();
  require(memory.read(relocated_qram) == pattern, "INITEE moves the same 1536 QRAM bytes");
  reject([&memory] { static_cast<void>(memory.read(reset_qram)); }, "Old QRAM mapping disappears");
  memory.write_register({.address = initee, .value = qram_at_2000 - 1});
  memory.retire_instruction();
  reject([&memory] { static_cast<void>(memory.read(relocated_qram)); }, "EEON disables QRAM");
  memory.write_register({.address = initee, .value = qram_at_2000});
  memory.retire_instruction();
  require(memory.read(relocated_qram) == pattern, "Disabling QRAM preserves its contents");
  constexpr unsigned qram_at_8000 = 0x81;
  memory.write_register({.address = initee, .value = qram_at_8000});
  memory.retire_instruction();
  reject([&memory] { static_cast<void>(memory.read(0x8000)); }, "Unknown QRAM must shadow flash");
  memory.write({.address = 0x8000, .value = inverse_pattern});
  require(memory.read(0x8000) == inverse_pattern, "QRAM takes priority over flash");
  memory.write_register({.address = initee, .value = 1});
  memory.retire_instruction();
  require(memory.read(0x8000) == pattern, "Flash contents remain intact under a QRAM overlay");
}

void test_register_mapping_and_flash_controls() {
  const auto flash = flash_fixture();
  blofeld::McuMemory memory(flash);
  constexpr unsigned registers_at_1000 = 0x10;
  constexpr unsigned register_offset_at_ram = 0x200;
  memory.write_register({.address = initrg, .value = registers_at_1000});
  memory.retire_instruction();
  const auto location = memory.locate(reset_ram);
  require(location.space == blofeld::McuSpace::registers &&
              location.offset == register_offset_at_ram,
          "Relocated registers shadow SMRAM");
  memory.reset();
  constexpr unsigned rom_high_only = 0x0f;
  memory.write_register({.address = misc, .value = rom_high_only});
  memory.retire_instruction();
  reject([&memory] { static_cast<void>(memory.read(0x4000)); }, "ROMHM disables lower flash alias");
  require(memory.read(0x8000) == pattern, "ROMHM leaves upper flash mapped");
  memory.reset();
  memory.write_register({.address = misc, .value = rom_high_only - 1});
  memory.retire_instruction();
  reject([&memory] { static_cast<void>(memory.read(0x8000)); }, "ROMON disables flash mapping");
}

void test_gpio_direction_and_unknown_inputs() {
  blofeld::McuGpio gpio;
  reject([&gpio] { static_cast<void>(gpio.read(q_data)); }, "Unknown pins do not read as zero");
  gpio.write({.address = 2, .value = all_pins});
  reject([&gpio] { static_cast<void>(gpio.read(0)); }, "Unknown output latch remains unknown");
  gpio.write({.address = 0, .value = pattern});
  require(gpio.read(0) == pattern, "Outputs read their written latch");
  gpio.write({.address = assignment_e, .value = 0x10});
  gpio.set_drive(blofeld::McuPort::e, {.mask = partial_pins, .levels = input_high});
  gpio.write({.address = direction_e, .value = 4});
  gpio.write({.address = port_e, .value = 4});
  require(gpio.read(port_e) == 12, "Port E combines driven input and output latch");
  gpio.write({.address = port_e, .value = 0});
  require(gpio.read(port_e) == input_high, "GPIO input does not follow an unrelated output latch");
  gpio.write({.address = direction_e, .value = all_pins});
  constexpr unsigned direction_mask_e = 0xfc;
  require(gpio.read(direction_e) == direction_mask_e, "IRQ and XIRQ cannot become outputs");
  gpio.write({.address = port_e, .value = all_pins});
  require(gpio.read(port_e) == direction_mask_e, "Input-only port E bits ignore output latch");
  constexpr unsigned direction_t = 0x242;
  constexpr unsigned data_t = 0x240;
  gpio.write({.address = direction_t, .value = all_pins});
  gpio.write({.address = data_t, .value = all_pins});
  require(gpio.read(data_t) == 15 && gpio.read(direction_t) == 15, "Only four port T pins exist");
}

void test_gpio_pull_devices_and_routing() {
  blofeld::McuGpio gpio;
  gpio.set_drive(blofeld::McuPort::q, {.floating = all_pins});
  reject([&gpio] { static_cast<void>(gpio.read(q_data)); }, "Floating inputs need a pull device");
  gpio.write({.address = q_pull_enable, .value = all_pins});
  require(gpio.read(q_data) == all_pins, "Pull-up polarity");
  gpio.write({.address = q_pull_select, .value = all_pins});
  require(gpio.read(q_data) == 0, "Pull-down polarity");
  constexpr unsigned high_pin = 0x80;
  constexpr unsigned other_pins = 0x7f;
  gpio.set_drive(blofeld::McuPort::q,
                 {.mask = high_pin, .levels = high_pin, .floating = other_pins});
  require(gpio.read(q_data) == high_pin, "External strong drive overrides weak pull-down");
  gpio.set_drive(blofeld::McuPort::q, {});
  gpio.write({.address = q_pull_enable, .value = 0});
  gpio.write({.address = routing, .value = 3});
  require(gpio.read(q_input) == 0, "64-pin MODRR tie-down overrides unused pad functions");
  gpio.write({.address = q_data, .value = all_pins});
  gpio.write({.address = q_direction, .value = all_pins});
  require(gpio.read(q_data) == all_pins && gpio.read(q_input) == 0,
          "Port latch and physical pad are distinct when pins are tied low");
}

void test_gpio_contention_and_reset() {
  blofeld::McuGpio gpio;
  gpio.write({.address = q_data, .value = inverse_pattern});
  gpio.write({.address = q_direction, .value = all_pins});
  gpio.set_drive(blofeld::McuPort::q, {.mask = 1, .levels = 1});
  require(gpio.read(q_data) == inverse_pattern, "Output data register reads its latch");
  reject([&gpio] { static_cast<void>(gpio.read(q_input)); },
         "Opposing strong pad drivers conflict");
  gpio.reset();
  require(gpio.read(q_direction) == 0, "Reset restores input direction");
  reject([&gpio] { static_cast<void>(gpio.read(q_input)); }, "Other external pins remain unknown");
  gpio.set_drive(blofeld::McuPort::q, {.mask = all_pins, .levels = pattern});
  require(gpio.read(q_input) == pattern, "Explicit external drives supply the physical input");
  reject([&gpio] { gpio.set_drive(blofeld::McuPort::q, {.mask = 1, .floating = 1}); },
         "A pin cannot be both driven and floating");
}

void test_instruction_mapping_boundary() {
  auto flash = flash_fixture();
  // Own test program: LDAA #$21; STAA $12; LDAA #$55; STAA $2000.
  constexpr std::array<std::uint8_t, 9> program{
      0x86, 0x21, 0x5a, 0x12, 0x86, 0x55, 0x7a, 0x20, 0x00};
  std::ranges::copy(program, flash.begin());
  constexpr unsigned reset_vector_offset = 0x7ffe;
  flash[reset_vector_offset] = 0x80;
  flash[reset_vector_offset + 1] = 0;
  blofeld::McuBus bus(flash);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  cpu.step();
  cpu.step();
  require(bus.read(initee) == qram_at_2000,
          "CPU retirement commits INITEE before the next instruction");
  cpu.step();
  cpu.step();
  require(bus.read(0x2000) == pattern,
          "Original instruction engine writes through the relocated map");
}

void test_bus_remapping_and_reset() {
  const auto flash = flash_fixture();
  blofeld::McuBus bus(flash);
  bus.write(reset_ram, pattern);
  bus.write(initee, qram_at_2000);
  bus.retire_instruction();
  bus.write(0x2000, inverse_pattern);
  bus.reset();
  require(bus.read(reset_ram) == pattern, "Board reset retains SMRAM bytes");
  reject([&bus] { static_cast<void>(bus.read(0x2000)); }, "Reset restores QRAM mapping");
  require(bus.read(initee) == 1, "Reset INITEE readback");
  constexpr unsigned reserved = 0xfe;
  bus.write(reserved, pattern);
  require(bus.read(reserved) == 0, "Documented reserved register range ignores writes");
  constexpr unsigned smartmedia_config = 0x11c;
  reject([&bus] { bus.write(smartmedia_config, 1); },
         "Unimplemented SmartMedia RAM mode must fail");
}

void check_context_fault(const auto &operation, const std::string_view access) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    const std::string_view message(fault.what());
    require(message.contains(access) && message.contains("pc=0x9123") &&
                message.contains("ccr=0x40"),
            "Peripheral faults retain operation/address and the caller's PC/CCR");
    return;
  }
  throw blofeld::Fault("Unknown peripheral access unexpectedly succeeded");
}

void test_peripheral_fault_context() {
  const auto flash = flash_fixture();
  blofeld::McuBus bus(flash);
  constexpr unsigned pc = 0x9123;
  constexpr unsigned ccr = 0x40;
  constexpr unsigned unsupported = 0x0d;
  constexpr unsigned usb_configuration = 0x310;
  bus.begin_instruction({.pc = pc, .ccr = ccr});
  check_context_fault([&bus] { static_cast<void>(bus.read(0)); }, "MCU read at 0x0000");
  check_context_fault([&bus] { bus.write(unsupported, 0); }, "MCU write at 0x000d");
  check_context_fault([&bus] { bus.write_memory({.address = usb_configuration, .size = 2}); },
                      "MCU operand write at 0x0310");
}
}

int main() {
  try {
    test_reset_map_and_unknown_ram();
    test_ram_remap_and_reset();
    test_qram_relocation_and_priority();
    test_register_mapping_and_flash_controls();
    test_gpio_direction_and_unknown_inputs();
    test_gpio_pull_devices_and_routing();
    test_gpio_contention_and_reset();
    test_bus_remapping_and_reset();
    test_instruction_mapping_boundary();
    test_peripheral_fault_context();
    std::cout << "MCU memory mapping, initialization, GPIO and reset checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
