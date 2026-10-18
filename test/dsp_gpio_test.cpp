// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_gpio.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr unsigned serial_mask = 0xfff;
constexpr unsigned dedicated_mask = 0x7ff;
constexpr unsigned pattern = 0x615;
constexpr std::array<blofeld::DspPort, 3> ports{
    blofeld::DspPort::c, blofeld::DspPort::e, blofeld::DspPort::f};
using Register = blofeld::DspGpioRegister;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

void test_reset_and_every_gpio_input_output() {
  for (const auto port : ports) {
    const unsigned width = port == blofeld::DspPort::f ? 11 : 12;
    for (unsigned index = 0; index < width; ++index) {
      blofeld::DspGpio gpio(port);
      const auto bit = 1U << index;
      require(gpio.read(Register::control) == 0 && gpio.read(Register::direction) == 0 &&
                  gpio.outputs().mask == 0 && gpio.observe_pads(bit).known == 0,
              "NXP tables 6-1/8-12/9-12 reset every implemented pin to disconnected");
      reject([&gpio, bit] { static_cast<void>(gpio.read(Register::data, bit)); },
             "A disconnected PDR bit is undefined, not a zero-ready input");
      gpio.write({.reg = Register::control, .value = bit});
      gpio.set_drive({.mask = bit, .levels = bit});
      require(gpio.read(Register::data, bit) == bit, "GPIO input follows an observed high level");
      gpio.set_drive({.mask = bit});
      require(gpio.read(Register::data, bit) == 0, "GPIO input follows an observed low level");
      gpio.write({.value = bit});
      require(gpio.read(Register::data, bit) == 0,
              "PDR writes cannot drive a pin configured input");
      gpio.set_drive({});
      reject([&gpio, bit] { static_cast<void>(gpio.read(Register::data, bit)); },
             "Releasing an input removes its qualification; no implicit pull-up is added");
      gpio.write({.reg = Register::control});
      gpio.write({.reg = Register::direction, .value = bit});
      require(gpio.outputs().mask == bit && gpio.outputs().known == 0,
              "A previous input/disconnected write does not qualify a future output latch");
      gpio.write({.value = bit});
      require(gpio.read(Register::data, bit) == bit && gpio.observe_pads(bit).levels == bit,
              "GPIO output reflects its explicitly written PDR value");
      reject([&gpio] { static_cast<void>(gpio.read(Register::data)); },
             "Reading one known bit cannot turn other disconnected bits into zeros");
    }
  }
}

void test_output_contention_and_nonreset_latches() {
  for (const auto port : ports) {
    blofeld::DspGpio gpio(port);
    const unsigned mask = port == blofeld::DspPort::f ? dedicated_mask : serial_mask;
    gpio.write({.reg = Register::direction, .value = mask});
    gpio.write({.value = pattern});
    require(gpio.read(Register::data) == pattern && gpio.outputs().known == mask,
            "All output bits are known and reserved upper PDR bits read zero");
    gpio.set_drive({.mask = mask, .levels = pattern});
    reject([&gpio, mask] { gpio.set_drive({.mask = mask, .levels = pattern ^ mask}); },
           "Opposed external and GPIO drivers fail visibly");
    reject([&gpio, mask] { gpio.write({.value = pattern ^ mask}); },
           "A CPU write that opposes the observed external drive fails too");
    require(gpio.read(Register::data) == pattern, "Rejected conflicting changes preserve state");
    gpio.set_drive({});
    gpio.reset();
    require(gpio.outputs().mask == 0 && gpio.read(Register::direction) == 0,
            "Reset disconnects the pins and clears their direction");
    reject([&gpio] { static_cast<void>(gpio.read(Register::data)); },
           "Disconnected reads remain undefined even after a previously known PDR write");
    gpio.write({.reg = Register::direction, .value = mask});
    require(gpio.read(Register::data) == pattern, "PDR latches are not reset with PCR/PRR");
  }
}

void test_mux_ownership_and_reserved_modes() {
  for (const auto port : {blofeld::DspPort::c, blofeld::DspPort::e}) {
    blofeld::DspGpio gpio(port);
    gpio.write({.reg = Register::control, .value = 10});
    gpio.write({.reg = Register::direction, .value = 12});
    gpio.write({.value = 4});
    gpio.set_drive({.mask = 10, .levels = 10});
    require(gpio.outputs().mask == 4 && gpio.outputs().peripheral == 8 &&
                gpio.read(Register::data, 6) == 6,
            "PCR/PRR table selects disconnected, input, output and ESAI independently per bit");
    reject([&gpio] { static_cast<void>(gpio.read(Register::data, 8)); },
           "An ESAI-owned PDR bit is not fabricated from its external input");
    require(gpio.observe_pads(8).known == 0, "Unmodeled ESAI pads remain unknown");
  }
  blofeld::DspGpio dedicated(blofeld::DspPort::f);
  dedicated.write({.reg = Register::control, .value = 1});
  reject([&dedicated] { dedicated.write({.reg = Register::direction, .value = 1}); },
         "Port F PCR=PRR=1 is reserved, not an ESAI selection");
  require(dedicated.read(Register::direction) == 0,
          "Rejected reserved function leaves PRRF intact");
  for (const auto port : ports) {
    blofeld::DspGpio gpio(port);
    const auto reserved = port == blofeld::DspPort::f ? 0x800U : 0x1000U;
    for (const auto reg : {Register::data, Register::direction, Register::control}) {
      reject([&gpio, reserved, reg] { gpio.write({.reg = reg, .value = reserved}); },
             "Undocumented nonzero writes to reserved bits require qualification");
    }
    reject([&gpio] { gpio.set_drive({.mask = 1, .levels = 2}); }, "Drive levels must be driven");
    reject([&gpio, reserved] { static_cast<void>(gpio.observe_pads(reserved)); },
           "Unimplemented pads cannot be observed as physical pins");
  }
}

void test_bus_spaces_and_port_f_biu_gate() {
  blofeld::DspBus bus;
  constexpr unsigned bcr = 0xfffffb;
  constexpr unsigned port_f_control = 0xfffff7;
  constexpr unsigned port_f_direction = 0xfffff6;
  constexpr unsigned port_c_direction = 0xffffbe;
  constexpr unsigned port_e_direction = 0xffff9e;
  constexpr unsigned bcr_reset = 0x1fffff;
  constexpr unsigned bcr_enabled = 0x01ffff;
  require(bus.read({.space = blofeld::DspSpace::x, .address = bcr}) == bcr_reset,
          "Family manual BCR reset fields select maximum wait states");
  reject(
      [&bus] {
        static_cast<void>(bus.read({.space = blofeld::DspSpace::y, .address = port_f_control}));
      },
      "Port F register access requires the documented BIU configuration");
  bus.write({.space = blofeld::DspSpace::x, .address = bcr, .value = bcr_enabled});
  reject([&bus] { static_cast<void>(bus.read({.space = blofeld::DspSpace::x, .address = bcr})); },
         "Programming wait states does not observe the read-only BIU bus-master status");
  bus.write({.space = blofeld::DspSpace::y, .address = port_f_direction, .value = dedicated_mask});
  bus.write({.space = blofeld::DspSpace::x, .address = port_c_direction, .value = serial_mask});
  bus.write({.space = blofeld::DspSpace::y, .address = port_e_direction, .value = serial_mask});
  require(bus.gpio(blofeld::DspPort::f).outputs().mask == dedicated_mask &&
              bus.gpio(blofeld::DspPort::c).outputs().mask == serial_mask &&
              bus.gpio(blofeld::DspPort::e).outputs().mask == serial_mask,
          "X port C and Y ports E/F route to separate pin banks");
  reject([&bus] { bus.write({.space = blofeld::DspSpace::x, .address = bcr, .value = 0}); },
         "Other BIU modes are unsupported rather than silently accepted");
  bus.reset_peripherals();
  require(bus.read({.space = blofeld::DspSpace::y, .address = port_f_direction}) == 0 &&
              bus.gpio(blofeld::DspPort::c).outputs().mask == 0,
          "Peripheral reset disconnects all three GPIO banks");
  bus.reset();
  require(bus.read({.space = blofeld::DspSpace::x, .address = bcr}) == bcr_reset,
          "Hardware reset also restores the core BIU control");
  reject(
      [&bus] {
        static_cast<void>(bus.read({.space = blofeld::DspSpace::y, .address = port_f_control}));
      },
      "Reset cannot retain an earlier Port F access qualification");
}
}

int main() {
  try {
    test_reset_and_every_gpio_input_output();
    test_output_contention_and_nonreset_latches();
    test_mux_ownership_and_reserved_modes();
    test_bus_spaces_and_port_f_biu_gate();
    std::cout << "DSP GPIO modes, unknown pads/latches, contention, reset and BIU gate passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
