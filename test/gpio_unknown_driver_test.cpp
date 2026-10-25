// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_gpio.hpp"
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
using blofeld::McuPort;
using Reg = blofeld::DspGpioRegister;
namespace hc12 = blofeld::hc12;
constexpr unsigned reserved = 0x1000;
constexpr unsigned pc = 0x8123;
constexpr unsigned ccr = 0x90;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned other_pins = 0xfe;
constexpr std::array<std::uint8_t, 32768> flash{};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

void rejects(const auto &operation, const std::string_view diagnostic) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    require(std::string_view(fault.what()).contains(diagnostic), "Unexpected driver diagnostic");
    return;
  }
  throw blofeld::Fault("Missing driver diagnostic");
}

void mcu_unknown_and_partial_updates() {
  blofeld::McuGpio gpio;
  gpio.write({.address = hc12::perq, .value = 3});
  gpio.set_drive(McuPort::q, {.mask = other_pins, .levels = other_pins, .unknown = 1});
  require(gpio.observe_pads(McuPort::q, 3).known == 2 &&
              gpio.observe_pads(McuPort::q, 3).levels == 2,
          "A driven unknown is neither a strong zero nor a weakly pulled-up input");
  rejects([&] { static_cast<void>(gpio.read(hc12::ptq)); }, "mask=0x01");
  gpio.update_drive(McuPort::q, 1, {.floating = 1});
  require(gpio.observe_pads(McuPort::q, 3).known == 3 &&
              gpio.observe_pads(McuPort::q, 3).levels == 3,
          "Releasing one unknown driver admits its pull without changing the other driver");
  gpio.update_drive(McuPort::q, 2, {.unknown = 2});
  rejects([&] { gpio.update_drive(McuPort::q, 1, {.unknown = 2}); }, "partial");
  require(gpio.observe_pads(McuPort::q, 3).known == 1 &&
              gpio.observe_pads(McuPort::q, 3).levels == 1,
          "Partial updates preserve unknown peers outside the selected mask");
  gpio.write({.address = hc12::ptq, .value = 3});
  gpio.write({.address = hc12::ddrq, .value = 3});
  require(gpio.outputs(McuPort::q) == blofeld::PinOutputs{.mask = 3, .known = 3, .levels = 3} &&
              gpio.observe_pads(McuPort::q, 3).known == 1,
          "Local output intent stays known while an unknown peer makes the shared pad unknown");
  require(gpio.read(hc12::ptq) == byte_mask,
          "Architectural MCU output-latch reads do not sample the uncertain output pad");
  gpio.reset();
  gpio.write({.address = hc12::ptq, .value = 3});
  gpio.write({.address = hc12::ddrq, .value = 3});
  require(gpio.observe_pads(McuPort::q, 3).known == 1,
          "Reset preserves peer drive state independently of local output latches");
}

void dsp_unknown_and_turnaround() {
  blofeld::DspGpio gpio(blofeld::DspPort::c);
  gpio.write({.reg = Reg::control, .value = 3});
  gpio.set_drive({.mask = 2, .levels = 2, .unknown = 1});
  require(gpio.observe_pads(3).known == 2 && gpio.observe_pads(3).levels == 2,
          "DSP input qualification must retain the peer's driven-unknown bit");
  rejects([&] { static_cast<void>(gpio.read(Reg::data, 3)); }, "Unknown DSP GPIO data read");
  gpio.set_drive({.floating = 2, .unknown = 1});
  require(gpio.observe_pads(3).known == 2 && gpio.observe_pads(3).levels == 0,
          "Only the explicitly floating pin can use Port C's weak pull-down");
  gpio.write({.reg = Reg::control});
  gpio.write({.reg = Reg::direction, .value = 3});
  gpio.write({.value = 3});
  require(gpio.outputs().known == 3 && gpio.observe_pads(3).known == 2,
          "A known local output cannot determine an unknown peer's electrical contribution");
  rejects([&] { static_cast<void>(gpio.read(Reg::data, 3)); }, "Unknown");
  gpio.reset();
  gpio.write({.reg = Reg::direction, .value = 3});
  require(gpio.observe_pads(3).known == 2, "Reset retains the unknown external driver");
  gpio.set_drive({.mask = 1, .levels = 1});
  require(gpio.read(Reg::data, 3) == 3, "Observing the peer high resolves the previous unknown");
  rejects([&] { gpio.set_drive({.mask = 1}); }, "contention");
  rejects([&] { gpio.write({.value = 2}); }, "contention");
  require(gpio.read(Reg::data, 3) == 3, "Rejected known opposition preserves the original latch");
}

void invalid_driver_masks() {
  blofeld::McuGpio mcu;
  blofeld::DspGpio dsp(blofeld::DspPort::c);
  rejects([&] { mcu.set_drive(McuPort::q, {.mask = 1, .unknown = 1}); }, "masks");
  rejects([&] { mcu.set_drive(McuPort::q, {.floating = 1, .unknown = 1}); }, "masks");
  rejects([&] { mcu.set_drive(McuPort::j, {.unknown = 8}); }, "masks");
  rejects([&] { dsp.set_drive({.mask = 1, .unknown = 1}); }, "drive");
  rejects([&] { dsp.set_drive({.floating = 1, .unknown = 1}); }, "drive");
  rejects([&] { dsp.set_drive({.unknown = reserved}); }, "drive");
  dsp.write({.reg = Reg::control, .value = 1});
  dsp.write({.reg = Reg::direction, .value = 1});
  dsp.set_drive({.unknown = 1});
  rejects([&] { dsp.validate_peripheral_drive({.mask = 1, .levels = 1}); }, "Unknown external");
  rejects([&] { dsp.validate_peripheral_drive({.unknown = 1}); }, "Invalid DSP peripheral");
}

void context_and_clone() {
  blofeld::McuBus bus(flash);
  bus.begin_instruction({.pc = pc, .ccr = ccr});
  bus.set_pin_drive(McuPort::q, {.unknown = 1});
  const auto copy = bus.clone();
  bus.set_pin_drive(McuPort::q, {.mask = 1, .levels = 1});
  require(bus.observe_pads(McuPort::q, 1).known == 1 &&
              copy->observe_pads(McuPort::q, 1).known == 0,
          "A board clone retains its own unknown peer after the original peer becomes known");
  rejects([&] { static_cast<void>(copy->read(hc12::ptq)); }, "pc=0x8123 ccr=0x90");
  rejects([&] { static_cast<void>(copy->read(hc12::ptq)); }, "0x0260");
}
}

int main() {
  try {
    mcu_unknown_and_partial_updates();
    dsp_unknown_and_turnaround();
    invalid_driver_masks();
    context_and_clone();
    std::cout << "Driven unknowns retain pad uncertainty; known opposition remains a fault.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
