// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hcs08_gpio.hpp"
#include "hcs08_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <utility>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr auto contract = Hcs08GpioContract::candidate_gt8a;
constexpr auto port = Hcs08GpioPort::e;
constexpr unsigned data = 0x10, pull = 0x11, slew = 0x12, direction = 0x13;
constexpr unsigned width = 0x3f, latch = 0x25, mixed = 0x19, inputs = 0x3c;
constexpr ExecutionContext context{.pc = 0x8123, .ccr = 0x68};

GpioNetDriver subset(GpioNetDriver driver, const unsigned mask) {
  driver.active &= mask;
  driver.known &= mask;
  driver.levels &= mask;
  driver.pull_up &= mask;
  driver.pull_down &= mask;
  return driver;
}

void registers() {
  struct Spec {
    Hcs08GpioPort port;
    unsigned base, mask;
  };

  constexpr std::array<Spec, 5> specs{{{.port = Hcs08GpioPort::a, .base = 0, .mask = 255},
                                       {.port = Hcs08GpioPort::b, .base = 4, .mask = 255},
                                       {.port = Hcs08GpioPort::c, .base = 8, .mask = 255},
                                       {.port = Hcs08GpioPort::d, .base = 12, .mask = 31},
                                       {.port = Hcs08GpioPort::e, .base = 16, .mask = 63}}};
  Hcs08Gpio gpio(contract);
  for (const auto spec : specs) {
    for (unsigned reg = 0; reg < 4; ++reg) {
      for (unsigned value = 0; value < 256; ++value) {
        gpio.reset();
        Hcs08GpioRegisters expected;
        const auto masked = static_cast<std::uint8_t>(value & spec.mask);
        switch (reg) {
          case 0: expected.latch = masked; break;
          case 1: expected.pull_enable = masked; break;
          case 2: expected.slew_enable = masked; break;
          case 3: expected.direction = masked; break;
          default: throw Fault("Invalid own register expectation");
        }
        gpio.write({.address = static_cast<std::uint16_t>(spec.base + reg),
                    .value = static_cast<std::uint8_t>(value)},
                   context);
        require(gpio.state(spec.port) == expected,
                "Independent20-register/all-byte expectations retain only implemented pin fields");
        if (reg != 0) {
          require(gpio.read(static_cast<std::uint16_t>(spec.base + reg), context) == masked,
                  "Control reads return actual stored fields, including zero non-pin positions");
        }
        gpio.write({.address = static_cast<std::uint16_t>(spec.base + 3),
                    .value = static_cast<std::uint8_t>(spec.mask)},
                   context);
        require(gpio.read(static_cast<std::uint16_t>(spec.base), context) == expected.latch,
                "Output reads use the latch without querying external voltage");
      }
    }
  }
}

void pads_and_drivers() {
  Hcs08Gpio gpio(contract);
  require(gpio.state(port) == Hcs08GpioRegisters{} && gpio.driver(port, width).active == 0 &&
              gpio.driver(port, width).pull_up == 0,
          "Reset clears latches/DD/PE/SE and supplies no output or passive driver");
  gpio.write({.address = data, .value = latch}, context);
  gpio.write({.address = pull, .value = width}, context);
  require(rejected([&] { static_cast<void>(gpio.read(data, context)); }).contains("Detached"),
          "An enabled pull cannot establish unknown external connectivity as floating");
  gpio.write({.address = direction, .value = 3}, context);
  gpio.write({.address = slew, .value = width}, context);
  const auto drive = gpio.driver(port, width);
  require(drive.active == 3 && drive.known == 3 && drive.levels == 1 && drive.pull_up == inputs &&
              drive.pull_down == 0 && drive.reduced == 0 && gpio.driver(port, 1).active == 1 &&
              gpio.driver(port, 1).pull_up == 0,
          "Explicit GPIO ownership exposes actual push-pull outputs and input pulls; slew is not "
          "reduced output strength");
  unsigned calls = 0;
  constexpr unsigned external_high = 0x18;
  GpioNetDriver external{.active = inputs, .known = inputs, .levels = external_high};
  gpio.attach_pad_source(port, [&](const unsigned mask) {
    ++calls;
    return resolve_gpio_net(mask, subset(gpio.driver(port, width), mask), subset(external, mask));
  });
  require(gpio.read(data, context) == mixed && calls == 1,
          "Mixed data read combines output latch01 with real resolved input18");
  external = {}; // Explicitly observed floating net; internal pulls now determine its voltage.
  require(gpio.read(data, context) == (inputs | 1U),
          "Observed floating inputs resolve through pulls");
  external = {.active = inputs};
  const auto error = rejected([&] { static_cast<void>(gpio.read(data, context)); });
  require(error.contains("address=0x0010") && error.contains("pc=0x8123") &&
              error.contains("ccr=0x0068") && error.contains("mask=0x00003c"),
          "Unknown active external driver defeats weak pull knowledge with full native context");
  external = {.pull_down = inputs};
  require(rejected([&] { static_cast<void>(gpio.read(data, context)); }).contains("conflicts"),
          "Opposing passive pulls remain unresolved");
  gpio.write({.address = direction, .value = width}, context);
  const auto before = calls;
  require(gpio.read(data, context) == latch && calls == before,
          "Even an unresolved physical net cannot replace documented output-latch readback");
}

void copies_and_failures() {
  Hcs08Gpio gpio(contract);
  gpio.attach_pad_source(port, [](const unsigned mask) { return GpioPadSample{.known = mask}; });
  gpio.write({.address = data, .value = latch}, context);
  auto copied = gpio;
  auto moved = std::move(copied);
  require(rejected([&] { static_cast<void>(moved.read(data, context)); }).contains("Detached"),
          "Copied/moved chip state cannot retain a callback into another board");
  moved.attach_pad_source(
      port, [](const unsigned mask) { return GpioPadSample{.known = mask, .levels = mask & 8U}; });
  require(moved.read(data, context) == 8 && gpio.read(data, context) == 0,
          "Fresh board pad sources remain independent");
  gpio.reset();
  require(gpio.state(port) == Hcs08GpioRegisters{} && gpio.read(data, context) == 0,
          "Reset clears chip registers while retaining the same physical connection");
  constexpr std::array<unsigned, 4> unsupported{20, 24, 0x1802, 0xffff};
  for (const auto address : unsupported) {
    const auto before = gpio.state(port);
    require(rejected([&] {
              gpio.write({.address = static_cast<std::uint16_t>(address), .value = 1}, context);
            }).contains("GPIO write") &&
                rejected([&] {
                  static_cast<void>(gpio.read(static_cast<std::uint16_t>(address), context));
                }).contains("pc=0x8123") &&
                gpio.state(port) == before,
            "G/BDC, unrelated peripherals and arbitrary addresses are not silent GPIO storage");
  }
  require(rejected([] {
            const Hcs08Gpio invalid(Hcs08GpioContract::unqualified);
          }).contains("explicit") &&
              rejected([&] { static_cast<void>(gpio.driver(port, 0x80)); }).contains("ownership"),
          "Unknown part contract and invalid ownership mask refuse");
}
} // namespace

int main() {
  try {
    registers();
    pads_and_drivers();
    copies_and_failures();
    std::cout
        << "HCS08 GPIO register, pin knowledge, driver and checkpoint binding effects verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
