// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

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
              blofeld::PinOutputs{.mask = upper, .known = upper, .levels = local_levels},
          "Driver export is the literal local latch/direction, even during opposed peer drive");
  rejects([&] { static_cast<void>(gpio.observe_pads(McuPort::q, byte_mask)); }, "contention");
  gpio.set_drive(McuPort::q, {.mask = lower, .levels = 3});
  require(gpio.observe_pads(McuPort::q, byte_mask).levels == combined_levels,
          "Independent upper local and lower peer drivers resolve to the actual pad byte");
  gpio.write({.address = reg::ddrq, .value = 0});
  require(gpio.outputs(McuPort::q) == blofeld::PinOutputs{},
          "Native direction release withdraws every local data driver");
}

void unknown_and_peripheral_drivers() {
  blofeld::McuGpio gpio;
  gpio.write({.address = reg::ddra, .value = 3});
  gpio.set_drive(McuPort::a, {.mask = 3, .levels = 3});
  require(gpio.outputs(McuPort::a) == blofeld::PinOutputs{.mask = 3},
          "A matching peer cannot qualify cold unknown local output latches");
  gpio.write({.address = reg::ddrq, .value = byte_mask});
  gpio.write({.address = reg::ptq, .value = mixed});
  gpio.set_function(
      McuPort::q,
      {.inputs = lower, .outputs = upper, .levels = peripheral_level, .unknown = unknown_bit});
  require(gpio.outputs(McuPort::q) == blofeld::PinOutputs{.mask = upper,
                                                          .known = peripheral_known,
                                                          .levels = peripheral_level},
          "Peripheral ownership suppresses GPIO direction and preserves an unknown driver bit");
  gpio.set_function(McuPort::q, {});
  require(gpio.outputs(McuPort::q) ==
              blofeld::PinOutputs{.mask = byte_mask, .known = byte_mask, .levels = mixed},
          "Releasing alternate ownership exposes the original literal GPIO latch");
  rejects([&] { static_cast<void>(gpio.outputs(McuPort::e)); }, "ECLK");
  gpio.write({.address = reg::modrr, .value = 3});
  rejects([&] { static_cast<void>(gpio.outputs(McuPort::q)); }, "package");
}

void board_observation() {
  blofeld::McuBus bus(flash);
  bus.begin_instruction({.pc = context_pc, .ccr = context_ccr});
  rejects([&] { static_cast<void>(bus.gpio_outputs(McuPort::e)); },
          "output observation at 0x0008",
          "pc=0x8123 ccr=0x90");
  bus.write(reg::ptq, mixed);
  bus.write(reg::ddrq, upper);
  bus.set_pin_drive(McuPort::q, {.mask = lower, .levels = 3});
  const auto revision = bus.revision();
  const auto code_revision = bus.code_revision();
  const auto before = bus.power_off_flash_image();
  const auto output = bus.gpio_outputs(McuPort::q);
  require(output == blofeld::PinOutputs{.mask = upper, .known = upper, .levels = local_levels} &&
              bus.observe_pads(McuPort::q, byte_mask).levels == combined_levels,
          "The board observes local drivers while the actual combined pad byte stays intact");
  const auto after = bus.power_off_flash_image();
  require(bus.revision() == revision && bus.code_revision() == code_revision &&
              before.bytes == after.bytes && before.known == after.known &&
              before.fresh == after.fresh,
          "Driver observation changes no memory revision or persistent state");
  const auto copy = bus.clone();
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
    board_observation();
    std::cout << "Local MCU drivers remain separate from peer levels, weak pulls and pad faults.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
