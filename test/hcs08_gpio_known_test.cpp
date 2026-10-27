// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr std::array<unsigned, 5> pins{0xff, 0xff, 0xff, 0x1f, 0x3f};
constexpr unsigned partial_pin_known = 0xc8;
constexpr unsigned alternating = 0x55, opposite = 0xaa, expected_cases = 15360;
constexpr ExecutionContext context{.pc = 0xe04e, .ccr = 0x68};

struct Input {
  unsigned port, direction, latch;
  GpioPadSample sample;
};

Hcs08KnownByte expected(const Input &input) {
  unsigned always = byte_mask;
  unsigned ever = 0;
  for (unsigned candidate = 0; candidate < byte_values; ++candidate) {
    if ((candidate & input.sample.known) != input.sample.levels) { continue; }
    unsigned read = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
      const unsigned mask = 1U << bit;
      if ((pins[input.port] & mask) == 0) { continue; }
      const unsigned physical = ((input.direction & mask) != 0 ? input.latch : candidate) & mask;
      read |= physical;
    }
    always &= read;
    ever |= read;
  }
  return {.value = static_cast<std::uint8_t>(always),
          .known = static_cast<std::uint8_t>(byte_mask ^ (always ^ ever))};
}

bool same(const Hcs08KnownByte a, const Hcs08KnownByte b) {
  return a.value == b.value && a.known == b.known;
}

void matrix() {
  unsigned cases = 0;
  for (unsigned port = 0; port < pins.size(); ++port) {
    Hcs08Gpio gpio(Hcs08GpioContract::candidate_gt8a);
    GpioPadSample sample;
    unsigned reads = 0;
    unsigned wanted = 0;
    gpio.attach_pad_source(static_cast<Hcs08GpioPort>(port), [&](const unsigned mask) {
      require(mask == wanted, "Only actual input pin positions request an external sample");
      ++reads;
      return sample;
    });
    for (unsigned direction = 0; direction < byte_values; ++direction) {
      const auto address = static_cast<std::uint16_t>(port * 4U);
      const unsigned latch = direction ^ byte_mask;
      gpio.write({.address = address, .value = static_cast<std::uint8_t>(latch)}, context);
      gpio.write({.address = static_cast<std::uint16_t>(address + 3U),
                  .value = static_cast<std::uint8_t>(direction)},
                 context);
      wanted = pins[port] & ~direction;
      for (const auto known : {0U, wanted, wanted & alternating, wanted & opposite}) {
        for (const auto levels : {0U, known, known & alternating}) {
          sample = {.known = known, .levels = levels};
          const unsigned prior = reads;
          require(
              same(gpio.read_known_byte(address, context),
                   expected(
                       {.port = port, .direction = direction, .latch = latch, .sample = sample})),
              "Known GPIO bits agree with every possible physical byte and output latch");
          require(reads == prior + (wanted != 0 ? 1U : 0U),
                  "A raw GPIO read samples each input set once and never samples output-only pins");
          const auto register_address = static_cast<std::uint16_t>(address + 3U);
          require(same(gpio.read_known_byte(register_address, context),
                       Hcs08KnownByte{.value = static_cast<std::uint8_t>(direction & pins[port])}),
                  "Direction registers retain complete readback knowledge");
          ++cases;
        }
      }
    }
  }
  require(cases == expected_cases, "All five ports and byte-wide direction patterns are qualified");
}

void refusals_and_copy() {
  constexpr unsigned data = 0x10;
  Hcs08Gpio gpio(Hcs08GpioContract::candidate_gt8a);
  require(rejected([&] {
            static_cast<void>(gpio.read_known_byte(data, context));
          }).contains("Detached"),
          "A detached source remains an actual failure, including raw knowledge reads");
  gpio.attach_pad_source(Hcs08GpioPort::e, [](const unsigned /*mask*/) {
    return GpioPadSample{.known = 8, .levels = 8};
  });
  require(same(gpio.read_known_byte(data, context),
               Hcs08KnownByte{.value = 8, .known = partial_pin_known}),
          "An observed pin does not invent levels for other implemented input pins");
  require(rejected([&] { static_cast<void>(gpio.read(data, context)); }).contains("voltage"),
          "Ordinary full-byte GPIO reads retain their unknown-input refusal");
  auto copy = gpio;
  require(rejected([&] {
            static_cast<void>(copy.read_known_byte(data, context));
          }).contains("Detached"),
          "Copied GPIO state requires reconnection to its own external source");
  copy.attach_pad_source(Hcs08GpioPort::e,
                         [](const unsigned mask) { return GpioPadSample{.conflicts = mask}; });
  const auto failure = rejected([&] { static_cast<void>(copy.read_known_byte(data, context)); });
  require(failure.contains("GPIO masked read") && failure.contains("address=0x0010") &&
              failure.contains("pc=0xe04e") && failure.contains("conflicts=0x00003f"),
          "Opposed inputs remain visible contextual failures instead of masked successful reads");
}
} // namespace

int main() {
  try {
    matrix();
    refusals_and_copy();
    std::cout << "GPIO knowledge retains unresolved inputs and strict ordinary reads.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
