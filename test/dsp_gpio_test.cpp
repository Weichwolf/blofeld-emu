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

void test_explicitly_floating_port_c_pads() {
  constexpr unsigned width = 12;
  for (unsigned index = 0; index < width; ++index) {
    blofeld::DspGpio gpio(blofeld::DspPort::c);
    const auto bit = 1U << index;
    gpio.set_drive({.floating = bit});
    require(gpio.observe_pads(bit).known == bit && gpio.observe_pads(bit).levels == 0,
            "NXP table 2-8 pulls each explicitly undriven PC0..11 pad down after reset");
    reject([&gpio, bit] { static_cast<void>(gpio.read(Register::data, bit)); },
           "A known disconnected pad voltage does not define its PDR register bit");
    gpio.write({.reg = Register::control, .value = bit});
    require(gpio.read(Register::data, bit) == 0,
            "Selecting GPIO input observes the documented pull-down on an undriven pad");
    gpio.set_drive({.mask = bit, .levels = bit});
    require(gpio.read(Register::data, bit) == bit,
            "A strong high input overrides the internal weak pull-down");
    gpio.set_drive({});
    reject([&gpio, bit] { static_cast<void>(gpio.read(Register::data, bit)); },
           "Unobserved external state must not be mistaken for a known floating input");
    gpio.set_drive({.floating = bit});
    gpio.write({.reg = Register::control});
    gpio.write({.reg = Register::direction, .value = bit});
    require(gpio.outputs().known == 0 && gpio.observe_pads(bit).known == 0,
            "A weak pull-down cannot qualify an unknown GPIO output latch");
    gpio.write({.value = bit});
    require(gpio.read(Register::data, bit) == bit,
            "An explicitly written high GPIO output overrides its weak pull-down");
    reject([&gpio, bit] { gpio.set_drive({.mask = bit}); },
           "Floating bias support must not hide strong output contention");
    gpio.reset();
    require(gpio.observe_pads(bit).known == bit && gpio.observe_pads(bit).levels == 0 &&
                gpio.outputs().mask == 0,
            "Reset retains explicit floating observations and disconnects the old output");
    reject([&gpio, bit] { static_cast<void>(gpio.read(Register::data, bit)); },
           "Reset still leaves disconnected Port C data reads undefined");
  }
}

void test_mixed_floating_driven_and_unknown_inputs() {
  constexpr unsigned driven = 0x505;
  constexpr unsigned floating = 0x202;
  constexpr unsigned known = 0x707;
  constexpr unsigned levels = 0x404;
  constexpr unsigned reserved = 0x1000;
  blofeld::DspGpio gpio(blofeld::DspPort::c);
  gpio.write({.reg = Register::control, .value = serial_mask});
  gpio.set_drive({.mask = driven, .levels = levels, .floating = floating});
  const auto check = [&gpio] {
    const auto pads = gpio.observe_pads(serial_mask);
    require(pads.known == known && pads.levels == levels &&
                gpio.read(Register::data, known) == levels,
            "Strong highs/lows and weak lows retain an independently specified known mask");
    reject([&gpio] { static_cast<void>(gpio.read(Register::data)); },
           "Other input bits remain unknown even beside qualified weak levels");
  };
  check();
  reject([&gpio] { gpio.set_drive({.mask = 1, .floating = 1}); },
         "An external pin cannot be strongly driven and floating at the same time");
  reject([&gpio] { gpio.set_drive({.levels = 1, .floating = 1}); },
         "Floating observations cannot carry a strong level");
  reject([&gpio] { gpio.set_drive({.floating = reserved}); },
         "Reserved bits cannot be declared floating pads");
  check();
  const auto saved = gpio;
  gpio.reset();
  gpio.write({.reg = Register::control, .value = serial_mask});
  check();
  gpio.set_drive({});
  require(gpio.observe_pads(serial_mask).known == 0,
          "Withdrawing all observations removes strong and weak input qualifications");
  gpio = saved;
  check();
}

void test_floating_does_not_qualify_unmodeled_functions() {
  blofeld::DspGpio gpio(blofeld::DspPort::c);
  gpio.write({.reg = Register::control, .value = 1});
  gpio.write({.reg = Register::direction, .value = 1});
  gpio.set_drive({.floating = 1});
  require(gpio.observe_pads(1).known == 0,
          "An unmodeled peripheral output cannot be replaced by a GPIO pull-down");
  reject([&gpio] { static_cast<void>(gpio.read(Register::data, 1)); },
         "Peripheral-owned PDR reads remain unqualified");
  reject([&gpio] { gpio.validate_peripheral_drive({.mask = 1, .floating = 1}); },
         "Peripheral output validation accepts strong outputs rather than floating declarations");
  gpio.validate_peripheral_drive({.mask = 1, .levels = 1});
  for (const auto port : {blofeld::DspPort::e, blofeld::DspPort::f, blofeld::DspPort::d}) {
    blofeld::DspGpio other(port);
    other.write({.reg = Register::control, .value = 1});
    other.set_drive({.floating = 1});
    require(other.observe_pads(1).known == 0,
            "This Port C qualification does not invent weak bias for other ports");
    reject([&other] { static_cast<void>(other.read(Register::data, 1)); },
           "Unqualified other-port floating input reads remain visible failures");
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

void test_symbolic_bit_access_preserves_unknown_state() {
  blofeld::DspBus bus;
  auto &gpio = bus.gpio(blofeld::DspPort::c);
  gpio.write({.reg = Register::direction, .value = 1});
  gpio.write({.value = 1});
  gpio.write({.reg = Register::direction, .value = 3});
  constexpr unsigned pdr = 0xffffbd;
  constexpr unsigned prr = 0xffffbe;
  using Operation = blofeld::DspBitOperation;

  struct Expected {
    Operation operation;
    bool old;
    unsigned level;
  };

  constexpr std::array<Expected, 4> cases{
      {{.operation = Operation::clear, .old = true, .level = 0},
       {.operation = Operation::set, .old = false, .level = 1},
       {.operation = Operation::change, .old = true, .level = 0},
       {.operation = Operation::test, .old = false, .level = 0}}};
  for (const auto &expected : cases) {
    const auto old = bus.access_peripheral_bit(
        {.space = blofeld::DspSpace::x, .address = pdr, .operation = expected.operation});
    const auto output = gpio.outputs();
    require(old.has_value() && *old == expected.old && output.mask == 3 && output.known == 1 &&
                output.levels == expected.level,
            "Only the selected output bit changes; an uninitialized output remains symbolic");
  }
  reject([&gpio] { static_cast<void>(gpio.read(Register::data)); },
         "Masked bit success does not initialize a whole GPIO data word");
  reject(
      [&bus] {
        static_cast<void>(bus.access_peripheral_bit({.space = blofeld::DspSpace::x,
                                                     .address = pdr,
                                                     .bit = 1,
                                                     .operation = Operation::set}));
      },
      "BSET needs the unknown previous bit for Carry even though its new value would be one");
  const auto old = bus.access_peripheral_bit(
      {.space = blofeld::DspSpace::x, .address = prr, .bit = 2, .operation = Operation::set});
  require(old.has_value() && !*old && gpio.outputs().mask == 7 && gpio.outputs().known == 1,
          "A control bit write preserves neighboring direction bits and cold data latches");
  constexpr unsigned fifo = 0xffff94;
  require(!bus.access_peripheral_bit({.space = blofeld::DspSpace::x, .address = fifo}).has_value(),
          "Declining a non-GPIO access performs no empty/destructive FIFO read");
  constexpr unsigned invalid_bit = 24;
  reject(
      [&bus] {
        static_cast<void>(bus.access_peripheral_bit(
            {.space = blofeld::DspSpace::x, .address = pdr, .bit = invalid_bit}));
      },
      "Out-of-range bit indices are rejected before shifting");
  reject([&gpio] { gpio.write({.value = 2, .mask = 1}); },
         "Masked write values cannot escape their mask");
  constexpr unsigned outside_word = 0x1000000;
  reject([&gpio] { gpio.write({.mask = outside_word}); }, "A mask cannot address a 25th GPIO bit");
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
    test_explicitly_floating_port_c_pads();
    test_mixed_floating_driven_and_unknown_inputs();
    test_floating_does_not_qualify_unmodeled_functions();
    test_mux_ownership_and_reserved_modes();
    test_symbolic_bit_access_preserves_unknown_state();
    test_bus_spaces_and_port_f_biu_gate();
    std::cout << "DSP GPIO modes, unknown pads/latches, contention, reset and BIU gate passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
