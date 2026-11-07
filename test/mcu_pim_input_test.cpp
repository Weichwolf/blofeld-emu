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
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned all = 0xff, first = 0x5a, second = 0xa5, retained = 0xc3;
constexpr std::array<std::uint8_t, 32768> flash{};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

void rejects(const auto &operation, const std::string_view expected) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains(expected), "Wrong register input diagnostic");
    return;
  }
  throw Fault("Unknown register input was accepted");
}

void published_inputs() {
  McuBus bus(flash);
  GpioPadSample published{.known = all, .levels = first};
  bus.attach_gpio_pad_source(McuPort::q, [&](const unsigned mask) {
    return GpioPadSample{.known = published.known & mask,
                         .levels = published.levels & mask,
                         .conflicts = published.conflicts & mask};
  });
  constexpr auto pin_input = hc12::ptq + 1;
  require(bus.read(hc12::ptq) == first && bus.read(pin_input) == first,
          "Data and input registers consume the current published byte");
  published.levels = second;
  require(bus.read(hc12::ptq) == second, "Publication needs no simulated settling edges");
  bus.write(hc12::ptq, retained);
  bus.write(hc12::ddrq, all);
  require(bus.read(hc12::ptq) == retained && bus.read(pin_input) == second,
          "The output latch and input register retain distinct register semantics");
  bus.write(hc12::ddrq, 0);
  published = {.known = 0x0f, .levels = 0x0a};
  require(bus.read_masked_byte({.address = hc12::ptq, .mask = 0x0f}) == 0x0a,
          "Masked access requires only the requested input bits");
  rejects([&] { static_cast<void>(bus.read(hc12::ptq)); }, "mask=0xf0");
  published = {.known = all ^ 2U, .conflicts = 2};
  rejects([&] { static_cast<void>(bus.read(pin_input)); }, "conflicts=0x02");
}
}

int main() {
  try {
    published_inputs();
    std::cout << "Functional MCU register inputs passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
