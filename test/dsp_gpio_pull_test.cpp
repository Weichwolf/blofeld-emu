// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_gpio.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
unsigned failures = 0;
unsigned observations = 0;

void expect(const bool condition,
            const std::string_view description,
            const blofeld::DspPort port,
            const unsigned pin) {
  ++observations;
  if (!condition) {
    ++failures;
    constexpr std::array names{"C", "E", "F", "D"};
    std::cerr << "FAIL port=" << names.at(static_cast<unsigned>(port)) << " pin=" << pin
              << " contract=" << description << '\n';
  }
}

bool rejects_read(const blofeld::DspGpio &gpio, const unsigned mask) {
  try {
    static_cast<void>(gpio.read(blofeld::DspGpioRegister::data, mask));
  } catch (const blofeld::Fault &) { return true; }
  return false;
}

void one_pin(const blofeld::DspPort port, const unsigned pin) {
  using namespace blofeld;
  const auto bit = 1U << pin;
  DspGpio gpio(port);
  gpio.write({.reg = DspGpioRegister::control, .value = bit});
  expect(rejects_read(gpio, bit), "unobserved connection remains unknown", port, pin);
  gpio.set_drive({.floating = bit});
  bool floating_low = false;
  try {
    floating_low = gpio.read(DspGpioRegister::data, bit) == 0;
  } catch (const Fault &) { floating_low = false; }
  expect(floating_low,
         "UM tables2-8..2-11 explicit Hi-Z input resolves through internal pull-down",
         port,
         pin);
  gpio.set_drive({.mask = bit, .levels = bit});
  expect(gpio.read(DspGpioRegister::data, bit) == bit,
         "driven high overrides passive pull-down",
         port,
         pin);
  gpio.set_drive({.unknown = bit});
  expect(
      rejects_read(gpio, bit), "unknown active driver is not resolved by passive bias", port, pin);
  gpio.set_drive({.floating = bit});
  gpio.write({.reg = DspGpioRegister::data, .value = 0});
  gpio.write({.reg = DspGpioRegister::control, .value = 0});
  gpio.write({.reg = DspGpioRegister::direction, .value = bit});
  expect(rejects_read(gpio, bit), "unknown enabled output latch remains unknown", port, pin);
  gpio.write({.reg = DspGpioRegister::data, .value = bit});
  expect(gpio.read(DspGpioRegister::data, bit) == bit,
         "known high output overrides weak bias",
         port,
         pin);
  gpio.write({.reg = DspGpioRegister::data, .value = 0});
  expect(
      gpio.read(DspGpioRegister::data, bit) == 0, "known low output remains driven low", port, pin);
}
} // namespace

int main() {
  try {
    using blofeld::DspPort;

    struct Port {
      DspPort port;
      unsigned pins;
    };

    constexpr unsigned serial_pins = 12;
    constexpr unsigned dedicated_pins = 11;
    constexpr unsigned digital_pins = 2;
    constexpr std::array ports{Port{.port = DspPort::c, .pins = serial_pins},
                               Port{.port = DspPort::e, .pins = serial_pins},
                               Port{.port = DspPort::f, .pins = dedicated_pins},
                               Port{.port = DspPort::d, .pins = digital_pins}};
    for (const auto port : ports) {
      for (unsigned pin = 0; pin < port.pins; ++pin) { one_pin(port.port, pin); }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  std::cout << "observations=" << observations << " failures=" << failures << '\n';
  return failures == 0 ? 0 : 1;
}
