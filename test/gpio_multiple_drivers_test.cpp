// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr std::array<GpioNetDriver, 8> states{GpioNetDriver{},
                                              {.active = 1, .known = 1},
                                              {.active = 1, .known = 1, .levels = 1},
                                              {.active = 1},
                                              {.pull_up = 1},
                                              {.pull_down = 1},
                                              {.active = 1, .known = 1, .reduced = 1},
                                              {.active = 1, .known = 1, .levels = 1, .reduced = 1}};
constexpr std::array<std::string_view, 8> pair_truth{
    "?01?1001", "00x?000x", "1x1?11x1", "????????", "101?1x01", "001?x001", "00x?000x", "1x1?11x1"};
constexpr unsigned wire_bits = 32, driver_count = 64, unknown_index = 17;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected an unqualified net/read failure");
}

GpioNetDriver shifted(const GpioNetDriver d, const std::uint32_t mask) {
  return {.active = d.active != 0 ? mask : 0,
          .known = d.known != 0 ? mask : 0,
          .levels = d.levels != 0 ? mask : 0,
          .reduced = d.reduced != 0 ? mask : 0,
          .pull_up = d.pull_up != 0 ? mask : 0,
          .pull_down = d.pull_down != 0 ? mask : 0};
}

void expect(const GpioPadSample result, const std::uint32_t mask, const char literal) {
  const auto known = literal == '0' || literal == '1' ? mask : 0;
  const auto high = literal == '1' ? mask : 0;
  const auto conflicts = literal == 'x' ? mask : 0;
  require(result.known == known && result.levels == high && result.conflicts == conflicts,
          "Resolved voltage matches the literal independent digital truth table");
}

void old_pairs_remain_exact() {
  for (unsigned bit = 0; bit < wire_bits; ++bit) {
    const std::uint32_t mask = std::uint32_t{1} << bit;
    for (unsigned first = 0; first < states.size(); ++first) {
      for (unsigned second = 0; second < states.size(); ++second) {
        const auto a = shifted(states.at(first), mask);
        const auto b = shifted(states.at(second), mask);
        const std::array drivers{a, b};
        expect(resolve_gpio_net(mask, a, b), mask, pair_truth.at(first).at(second));
        expect(resolve_gpio_net(mask, drivers), mask, pair_truth.at(first).at(second));
      }
    }
  }
}

void literal_three_driver_cases() {
  struct Case {
    std::array<unsigned, 3> states;
    char expected;
  };

  constexpr std::array cases{Case{.states = {0, 0, 0}, .expected = '?'},
                             Case{.states = {0, 4, 4}, .expected = '1'},
                             Case{.states = {0, 4, 5}, .expected = 'x'},
                             Case{.states = {1, 4, 5}, .expected = '0'},
                             Case{.states = {2, 4, 5}, .expected = '1'},
                             Case{.states = {1, 2, 4}, .expected = 'x'},
                             Case{.states = {3, 1, 4}, .expected = '?'},
                             Case{.states = {3, 2, 5}, .expected = '?'},
                             Case{.states = {1, 2, 3}, .expected = 'x'},
                             Case{.states = {6, 4, 0}, .expected = '0'},
                             Case{.states = {7, 5, 0}, .expected = '1'},
                             Case{.states = {6, 2, 4}, .expected = 'x'},
                             Case{.states = {7, 1, 5}, .expected = 'x'},
                             Case{.states = {1, 1, 1}, .expected = '0'},
                             Case{.states = {2, 2, 2}, .expected = '1'},
                             Case{.states = {3, 4, 5}, .expected = '?'},
                             Case{.states = {0, 5, 5}, .expected = '0'},
                             Case{.states = {3, 0, 0}, .expected = '?'}};
  for (unsigned bit = 0; bit < wire_bits; ++bit) {
    const std::uint32_t mask = std::uint32_t{1} << bit;
    for (const auto &c : cases) {
      std::array order{0U, 1U, 2U};
      do {
        const std::array drivers{shifted(states.at(c.states.at(order.front())), mask),
                                 shifted(states.at(c.states.at(order.at(1))), mask),
                                 shifted(states.at(c.states.at(order.back())), mask)};
        expect(resolve_gpio_net(mask, drivers), mask, c.expected);
      } while (std::ranges::next_permutation(order).found);
    }
  }
}

void many_drivers_and_invalid_inputs() {
  std::array<GpioNetDriver, driver_count> drivers{};
  drivers.fill({.pull_up = 1});
  expect(resolve_gpio_net(1, drivers), 1, '1');
  drivers.front() = {.active = 1, .known = 1};
  expect(resolve_gpio_net(1, drivers), 1, '0');
  drivers.at(unknown_index) = {.active = 1};
  expect(resolve_gpio_net(1, drivers), 1, '?');
  drivers.back() = {.active = 1, .known = 1, .levels = 1};
  expect(resolve_gpio_net(1, drivers), 1, 'x');
  const std::span<const GpioNetDriver> empty;
  expect(resolve_gpio_net(1, empty), 1, '?');
  constexpr std::array<GpioNetDriver, 7> invalid{GpioNetDriver{.active = 2},
                                                 {.known = 1},
                                                 {.levels = 1},
                                                 {.reduced = 1},
                                                 {.pull_up = 2},
                                                 {.pull_up = 1, .pull_down = 1},
                                                 {.active = 1, .pull_up = 1}};
  for (const auto d : invalid) {
    drivers.back() = d;
    static_cast<void>(rejects([&] { static_cast<void>(resolve_gpio_net(1, drivers)); }));
  }
}

GpioPadSample
sampled_reset_net(const McuBus &bus, const GpioNetDriver supervisor, const std::uint32_t mask) {
  const auto out = bus.gpio_outputs(McuPort::u);
  const auto bias = bus.gpio_biases(McuPort::u);
  const std::array drivers{GpioNetDriver{.active = out.mask & mask,
                                         .known = out.known & mask,
                                         .levels = out.levels & mask,
                                         .reduced = out.reduced_drive & mask,
                                         .pull_up = bias.up & mask,
                                         .pull_down = bias.down & mask},
                           GpioNetDriver{.pull_up = mask & 1U},
                           supervisor};
  return resolve_gpio_net(mask, drivers);
}

void mcu_reset_readback() {
  constexpr unsigned flash_bytes = 32768;
  constexpr unsigned direction = hc12::ptu + 2;
  constexpr unsigned pin_input = hc12::ptu + 1;
  constexpr unsigned diagnostic_pc = 0xb98d;
  constexpr unsigned diagnostic_ccr = 0x90;
  const std::array<std::uint8_t, flash_bytes> flash{};
  McuBus bus(flash);
  bus.begin_instruction({.pc = diagnostic_pc, .ccr = diagnostic_ccr});
  GpioNetDriver supervisor{.active = 1, .known = 1};
  bus.attach_gpio_pad_source(McuPort::u, [&](const auto mask) {
    auto selected = supervisor;
    selected.active &= mask;
    selected.known &= mask;
    selected.levels &= mask;
    return sampled_reset_net(bus, selected, mask);
  });
  require(bus.read_masked_byte({.address = hc12::ptu, .mask = 1}) == 0,
          "Declared external reset assertion resolves low before any MCU output");
  bus.write(hc12::ptu, 0);
  bus.write(direction, 1);
  supervisor = {};
  require(bus.read_masked_byte({.address = pin_input, .mask = 1}) == 0,
          "Actual MCU output-low dominates the declared passive DSP reset pull-up");
  supervisor = {.active = 1, .known = 1, .levels = 1};
  const auto conflict =
      rejects([&] { static_cast<void>(bus.read_masked_byte({.address = pin_input, .mask = 1})); });
  require(conflict.contains("0x0279") && conflict.contains("0xb98d") &&
              conflict.contains("conflicts=0x01"),
          "Opposing active drives fail at the real input register with supplied execution context");
  supervisor = {};
  bus.write(direction, 0);
  require(
      bus.read_masked_byte({.address = hc12::ptu, .mask = 1}) == 1,
      "Releasing the real MCU direction exposes passive reset readback without ready synthesis");
  static_cast<void>(rejects([&] { static_cast<void>(bus.read(hc12::ptu)); }));
  const auto frozen = bus.clone();
  static_cast<void>(rejects(
      [&] { static_cast<void>(frozen->read_masked_byte({.address = hc12::ptu, .mask = 1})); }));
  require(bus.read_masked_byte({.address = hc12::ptu, .mask = 1}) == 1,
          "Checkpoint clone drops external callbacks and cannot fill unknown neighboring pins");
}

void native_branch_reads_passive_net(const std::filesystem::path &specification) {
  constexpr unsigned entry = 0x8000;
  constexpr unsigned flash_base = 0x8000;
  constexpr unsigned branch_clear = 0x1f;
  constexpr unsigned loop_displacement = 0xfb;
  constexpr unsigned direction = hc12::ptu + 2;
  constexpr unsigned accumulator = 0x1234;
  constexpr unsigned index_x = 0x4567;
  constexpr unsigned index_y = 0x6789;
  constexpr unsigned stack = 0x1f00;
  const std::array<std::uint8_t, 5> code{branch_clear,
                                         static_cast<std::uint8_t>(hc12::ptu >> 8U),
                                         static_cast<std::uint8_t>(hc12::ptu & 0xffU),
                                         1,
                                         loop_displacement};
  std::array<std::uint8_t, 32768> flash{};
  std::ranges::copy(code, flash.begin() + (entry - flash_base));
  McuBus memory(flash);
  memory.attach_gpio_pad_source(
      McuPort::u, [&](const auto mask) { return sampled_reset_net(memory, {}, mask); });
  memory.write(hc12::ptu, 0);
  memory.write(direction, 1);
  std::vector<Hc12BusCycle> clocks;
  Hc12CycleBus timed(memory, [&](const auto cycle) { clocks.push_back(cycle); });
  Hc12 cpu(timed, specification);
  cpu.set_state({.pc = entry,
                 .d = accumulator,
                 .x = index_x,
                 .y = index_y,
                 .sp = stack,
                 .ccr = hc12::reset_ccr});
  static_cast<void>(cpu.step());
  require(cpu.state().pc == entry, "Native BRCLR loops while the actual MCU holds reset low");
  memory.write(direction, 0); // Declared event between completed native instructions.
  static_cast<void>(cpu.step());
  const auto result = cpu.state();
  require(result.pc == entry + code.size() && result.d == accumulator && result.x == index_x &&
              result.y == index_y && result.sp == stack && result.ccr == hc12::reset_ccr &&
              clocks.size() == 2 * code.size() && clocks.front().address == hc12::ptu &&
              clocks.at(code.size()).address == hc12::ptu,
          "Native selected-bit branch observes passive release in exactly five cycles without flag "
          "changes");
  static_cast<void>(rejects([&] { static_cast<void>(memory.read(hc12::ptu)); }));
}
}

int main(const int count, char **arguments) {
  try {
    old_pairs_remain_exact();
    literal_three_driver_cases();
    many_drivers_and_invalid_inputs();
    mcu_reset_readback();
    require(count == 2, "The native branch test requires a pinned HC12 specification");
    native_branch_reads_passive_net(arguments[1]);
    std::cout
        << "Multiple-driver GPIO truth tables, passive reset readback and strict unknowns pass\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
