// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
namespace reg = blofeld::hc12;
using blofeld::McuPort;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned alternating = 0x55;
constexpr unsigned upper = 0xf0;
constexpr unsigned lower = 0x0f;
constexpr unsigned mixed = 0xa5;
constexpr unsigned unknown_bit = 0x80;
constexpr unsigned local_levels = 0xa0;
constexpr unsigned combined_levels = 0xa3;
constexpr unsigned peripheral_known = 0x70;
constexpr unsigned peripheral_level = 0x20;
constexpr unsigned context_pc = 0x8123;
constexpr unsigned context_ccr = 0x90;
constexpr std::array<std::uint8_t, 32768> flash{};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

void rejects(const auto &operation,
             const std::string_view expected,
             const std::string_view context = {}) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    const std::string_view message(fault.what());
    require(message.contains(expected) && message.contains(context), "Unexpected GPIO diagnostic");
    return;
  }
  throw blofeld::Fault("Expected GPIO diagnostic is absent");
}

void separate_driver_and_pad() {
  blofeld::McuGpio gpio;
  gpio.write({.address = reg::rdrq, .value = byte_mask});
  gpio.write({.address = reg::perq, .value = byte_mask});
  gpio.set_drive(McuPort::q, {.floating = byte_mask});
  require(gpio.observe_pads(McuPort::q, byte_mask).levels == byte_mask &&
              gpio.outputs(McuPort::q) == blofeld::PinOutputs{},
          "Weak input pulls are readable pads, not exported strong output drivers");
  gpio.set_drive(McuPort::q, {.mask = byte_mask, .levels = alternating});
  require(gpio.observe_pads(McuPort::q, byte_mask).levels == alternating &&
              gpio.outputs(McuPort::q) == blofeld::PinOutputs{},
          "A peer's driven value must never feed back as a local output");
  gpio.write({.address = reg::ptq, .value = mixed});
  gpio.write({.address = reg::ddrq, .value = upper});
  require(gpio.outputs(McuPort::q) ==
              blofeld::PinOutputs{
                  .mask = upper, .known = upper, .levels = local_levels, .reduced_drive = upper},
          "Driver export is the literal local latch/direction, even during opposed peer drive");
  rejects([&] { static_cast<void>(gpio.observe_pads(McuPort::q, byte_mask)); }, "contention");
  gpio.set_drive(McuPort::q, {.mask = lower, .levels = 3});
  require(gpio.observe_pads(McuPort::q, byte_mask).levels == combined_levels,
          "Independent upper local and lower peer drivers resolve to the actual pad byte");
  gpio.write({.address = reg::ddrq, .value = 0});
  require(gpio.outputs(McuPort::q) == blofeld::PinOutputs{} && gpio.read(reg::rdrq) == byte_mask,
          "Native direction release withdraws drivers without changing strength selection");
}

void unknown_and_peripheral_drivers() {
  blofeld::McuGpio gpio;
  gpio.write({.address = reg::ddra, .value = 3});
  gpio.set_drive(McuPort::a, {.mask = 3, .levels = 3});
  require(gpio.outputs(McuPort::a) == blofeld::PinOutputs{.mask = 3},
          "A matching peer cannot qualify cold unknown local output latches");
  gpio.write({.address = reg::ddrq, .value = byte_mask});
  gpio.write({.address = reg::ptq, .value = mixed});
  gpio.write({.address = reg::rdrq, .value = byte_mask});
  gpio.set_function(
      McuPort::q,
      {.inputs = lower, .outputs = upper, .levels = peripheral_level, .unknown = unknown_bit});
  require(gpio.outputs(McuPort::q) == blofeld::PinOutputs{.mask = upper,
                                                          .known = peripheral_known,
                                                          .levels = peripheral_level,
                                                          .reduced_drive = upper},
          "Peripheral and unknown output latches retain their independent strength selection");
  gpio.set_function(McuPort::q, {});
  require(gpio.outputs(McuPort::q) == blofeld::PinOutputs{.mask = byte_mask,
                                                          .known = byte_mask,
                                                          .levels = mixed,
                                                          .reduced_drive = byte_mask},
          "Releasing alternate ownership exposes the original literal GPIO latch");
  gpio.write({.address = reg::rdrq, .value = 0});
  require(gpio.outputs(McuPort::q) ==
              blofeld::PinOutputs{.mask = byte_mask, .known = byte_mask, .levels = mixed},
          "Restoring full drive changes no direction, latch knowledge or level");
  gpio.reset();
  require(gpio.read(reg::rdrq) == 0 && gpio.outputs(McuPort::q) == blofeld::PinOutputs{},
          "Reset restores full-drive selection and releases the GPIO outputs");
  require(gpio.read(reg::pear) == 0x10 && gpio.outputs(McuPort::e) == blofeld::PinOutputs{},
          "Normal single-chip reset selects PE4 GPIO and releases all Port E drivers");
  gpio.write({.address = reg::pear, .value = 0});
  rejects([&] { static_cast<void>(gpio.outputs(McuPort::e)); }, "ECLK");
  gpio.write({.address = reg::modrr, .value = 3});
  rejects([&] { static_cast<void>(gpio.outputs(McuPort::q)); }, "package");
}

void output_publication_revision() {
  blofeld::McuGpio gpio;
  const auto initial = gpio.output_revision();
  gpio.set_drive(McuPort::q, {.floating = byte_mask});
  gpio.update_drive(McuPort::q, lower, {.mask = lower, .levels = lower});
  gpio.set_function(McuPort::q, {});
  static_cast<void>(gpio.outputs(McuPort::q));
  static_cast<void>(gpio.observe_pads(McuPort::q, lower));
  require(gpio.output_revision() == initial,
          "Peer input changes and observations do not republish local output ownership");
  const blofeld::PinFunction function{.outputs = lower, .levels = lower};
  gpio.set_function(McuPort::q, function);
  require(gpio.output_revision() != initial && gpio.outputs(McuPort::q).levels == lower,
          "A peripheral output transition invalidates publication without a register write");
  const auto peripheral = gpio.output_revision();
  gpio.set_function(McuPort::q, function);
  gpio.write({.address = reg::ptq + 1, .value = byte_mask});
  require(gpio.output_revision() == peripheral,
          "Unchanged peripheral ownership and read-only register writes are inert");
  gpio.set_function(McuPort::q, {});
  constexpr std::array writes{blofeld::ByteWrite{.address = reg::ptq, .value = mixed},
                              blofeld::ByteWrite{.address = reg::ddrq, .value = upper},
                              blofeld::ByteWrite{.address = reg::rdrq, .value = upper},
                              blofeld::ByteWrite{.address = reg::perq, .value = lower},
                              blofeld::ByteWrite{.address = reg::ppsq, .value = lower},
                              blofeld::ByteWrite{.address = reg::pucr, .value = 0},
                              blofeld::ByteWrite{.address = reg::pear, .value = 0},
                              blofeld::ByteWrite{.address = reg::modrr, .value = 3}};
  for (const auto write : writes) {
    const auto before = gpio.output_revision();
    gpio.write(write);
    require(gpio.output_revision() != before,
            "Latch, direction, drive, bias and routing writes invalidate local publication");
  }
  const auto before_reset = gpio.output_revision();
  gpio.reset();
  require(gpio.output_revision() != before_reset && gpio.outputs(McuPort::q).mask == 0,
          "Reset republishes released output ownership");
}

void board_observation() {
  blofeld::McuBus bus(flash);
  bus.begin_instruction({.pc = context_pc, .ccr = context_ccr});
  require(bus.read(reg::pear) == 0x10 && bus.gpio_outputs(McuPort::e) == blofeld::PinOutputs{},
          "Normal single-chip bus reset exposes GPIO without inventing a known input level");
  bus.write(reg::pear, 0);
  rejects([&] { static_cast<void>(bus.gpio_outputs(McuPort::e)); },
          "output observation at 0x0008",
          "pc=0x8123 ccr=0x90");
  bus.write(reg::ptq, mixed);
  bus.write(reg::ddrq, upper);
  bus.write(reg::rdrq, byte_mask);
  bus.set_pin_drive(McuPort::q, {.mask = lower, .levels = 3});
  const auto revision = bus.revision();
  const auto code_revision = bus.code_revision();
  const auto before = bus.power_off_flash_image();
  const auto output = bus.gpio_outputs(McuPort::q);
  require(output == blofeld::PinOutputs{.mask = upper,
                                        .known = upper,
                                        .levels = local_levels,
                                        .reduced_drive = upper} &&
              bus.observe_pads(McuPort::q, byte_mask).levels == combined_levels,
          "The board observes local drivers while the actual combined pad byte stays intact");
  const auto after = bus.power_off_flash_image();
  require(bus.revision() == revision && bus.code_revision() == code_revision &&
              before.bytes == after.bytes && before.known == after.known &&
              before.fresh == after.fresh,
          "Driver observation changes no memory revision or persistent state");
  const auto copy = bus.clone();
  bus.write(reg::rdrq, 0);
  require(copy->gpio_outputs(McuPort::q) == output &&
              bus.gpio_outputs(McuPort::q) ==
                  blofeld::PinOutputs{.mask = upper, .known = upper, .levels = local_levels},
          "Cloning preserves reduced strength independently of subsequent register writes");
  bus.write(reg::ddrq, 0);
  require(copy->gpio_outputs(McuPort::q) == output &&
              bus.gpio_outputs(McuPort::q) == blofeld::PinOutputs{},
          "A board snapshot keeps its driver state independently of subsequent direction writes");
}
}

int main() {
  try {
    separate_driver_and_pad();
    unknown_and_peripheral_drivers();
    output_publication_revision();
    board_observation();
    std::cout << "Local MCU drivers remain separate from peer levels, weak pulls and pad faults.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
