// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "runtime/gpio_pads.hpp"

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
    const auto sampled = gpio.observation_revision(McuPort::q);
    gpio.write(write);
    require(gpio.output_revision() != before && gpio.observation_revision(McuPort::q) != sampled,
            "Latch, direction, drive, bias and routing writes invalidate local publication");
  }
  const auto before_reset = gpio.output_revision();
  gpio.reset();
  require(gpio.output_revision() != before_reset && gpio.outputs(McuPort::q).mask == 0,
          "Reset republishes released output ownership");
}

void observation_revisions() {
  blofeld::McuGpio gpio;
  const auto initial = gpio.observation_revision(McuPort::q);
  const auto outputs = gpio.output_revision();
  require(initial.has_value(), "Locally owned port observations have an initial revision");
  gpio.set_drive(McuPort::q, {.floating = byte_mask});
  const auto driven = gpio.observation_revision(McuPort::q);
  require(driven != initial && gpio.output_revision() == outputs,
          "An actual input change invalidates observations without republishing local outputs");
  gpio.set_drive(McuPort::q, {.floating = byte_mask});
  static_cast<void>(gpio.observe_pads(McuPort::q, byte_mask));
  gpio.write({.address = reg::ptq + 1, .value = byte_mask});
  require(gpio.observation_revision(McuPort::q) == driven,
          "Unchanged drives, observations and ignored input writes retain their revision");
  gpio.set_function(McuPort::q, {.outputs = lower, .levels = lower});
  const auto peripheral = gpio.observation_revision(McuPort::q);
  require(peripheral != driven, "Peripheral output changes invalidate sampled port values");
  gpio.reset();
  require(gpio.observation_revision(McuPort::q) != peripheral,
          "A real GPIO reset invalidates previously sampled state");
  gpio.set_drive(McuPort::q, {});
  std::uint32_t levels = 0;
  gpio.attach_pad_source(McuPort::q, [&levels](const std::uint32_t mask) {
    return blofeld::GpioPadSample{.known = mask, .levels = levels & mask};
  });
  require(!gpio.observation_revision(McuPort::q) && gpio.observation_revision(McuPort::e),
          "An external reader has no version token; unrelated local ports remain versioned");
  require(gpio.observe_pads(McuPort::q, 1).levels == 0, "Sample the actual external reader");
  levels = 1;
  require(gpio.observe_pads(McuPort::q, 1).levels == 1 && !gpio.observation_revision(McuPort::q),
          "External values can change without any local register or drive write");
}

void repeated_pad_observations() {
  blofeld::McuGpio gpio;
  const auto expect = [&](const unsigned known, const unsigned levels) {
    for (const auto mask : {1U, lower, upper, byte_mask, 0U}) {
      const auto observed = gpio.observe_pads(McuPort::q, static_cast<std::uint8_t>(mask));
      require(observed.known == (known & mask) && observed.levels == (levels & mask) &&
                  observed.conflicts == 0,
              "Repeated differently masked observations retain the actual full port state");
    }
  };
  expect(0, 0);
  gpio.set_drive(McuPort::q, {.floating = byte_mask});
  gpio.write({.address = reg::perq, .value = byte_mask});
  expect(byte_mask, byte_mask);
  gpio.write({.address = reg::ppsq, .value = upper});
  expect(byte_mask, lower);
  gpio.update_drive(McuPort::q, lower, {.mask = lower, .levels = 3});
  expect(byte_mask, 3);
  gpio.write({.address = reg::ptq, .value = mixed});
  gpio.write({.address = reg::ddrq, .value = upper});
  expect(byte_mask, combined_levels);
  gpio.set_function(McuPort::q, {.outputs = upper, .levels = peripheral_level});
  expect(byte_mask, peripheral_level | 3);
  gpio.set_function(McuPort::q, {.outputs = upper, .unknown = upper});
  expect(lower, 3);
  gpio.set_function(McuPort::q, {});
  expect(byte_mask, combined_levels);
  const auto copy = gpio;
  gpio.reset();
  expect(lower, 3);
  require(copy.observe_pads(McuPort::q, byte_mask).levels == combined_levels,
          "A previously observed port copy remains independent after the original resets");
  gpio.set_drive(McuPort::q, {.floating = byte_mask});
  gpio.write({.address = reg::modrr, .value = 3});
  expect(byte_mask, 0);
  gpio.write({.address = reg::modrr, .value = 0});
  expect(0, 0);
}

void masked_contention_and_global_pulls() {
  blofeld::McuGpio gpio;
  gpio.write({.address = reg::ptq, .value = 0});
  gpio.write({.address = reg::ddrq, .value = upper});
  gpio.set_drive(McuPort::q, {.mask = byte_mask, .levels = unknown_bit | 3});
  require(gpio.observe_pads(McuPort::q, lower).levels == 3,
          "Opposition outside the requested bits cannot fault an unrelated input read");
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    rejects([&] { static_cast<void>(gpio.observe_pads(McuPort::q, unknown_bit)); }, "contention");
  }
  gpio.update_drive(McuPort::q, unknown_bit, {.floating = unknown_bit});
  require(gpio.observe_pads(McuPort::q, byte_mask).levels == 3,
          "Releasing the opposed driver immediately restores observable local zero");
  gpio.set_drive(McuPort::b, {.floating = byte_mask});
  require(gpio.observe_pads(McuPort::b, byte_mask).known == 0,
          "A released port with its global pull disabled stays unknown");
  gpio.write({.address = reg::pucr, .value = 2});
  require(gpio.observe_pads(McuPort::b, byte_mask).levels == byte_mask,
          "Global pull changes update a previously observed independent port");
  gpio.write({.address = reg::pucr, .value = 0});
  require(gpio.observe_pads(McuPort::b, byte_mask).known == 0,
          "Disabling the global pull immediately withdraws input knowledge");
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
    observation_revisions();
    repeated_pad_observations();
    masked_contention_and_global_pulls();
    board_observation();
    std::cout << "Local MCU drivers remain separate from peer levels, weak pulls and pad faults.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
