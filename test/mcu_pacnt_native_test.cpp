// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
constexpr unsigned entry = 0x8000;
constexpr unsigned program_size = 20;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned pulse_count = 0x62;
constexpr unsigned pulse_control = 0x60;
constexpr unsigned result_address = 0x1200;
constexpr unsigned timer_control = 0x46;
constexpr unsigned timer_flags = 0x4f;
constexpr unsigned channel_flags = 0x4e;
constexpr unsigned timer_on = 0x80;
constexpr unsigned timer_fast_clear = 0x90;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_cycles = 65536;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Product {
  std::uint16_t left;
  std::uint16_t right;
  std::uint16_t high;
  std::uint16_t low;
  std::uint8_t flags;
};

// Independent signed products and S12CPUV2 EMULS N/Z/C expectations.
// These are authored operand vectors, not extracted manufacturer programs.
constexpr std::array<Product, 6> products{{
    {.left = 0xfffe, .right = 3, .high = 0xffff, .low = 0xfffa, .flags = 0xd9},
    {.left = 0x4000, .right = 2, .high = 0, .low = 0x8000, .flags = 0xd1},
    {.left = 0x8000, .right = 0xffff, .high = 0, .low = 0x8000, .flags = 0xd1},
    {.left = 0x8000, .right = 0x7fff, .high = 0xc000, .low = 0x8000, .flags = 0xd9},
    {.left = 0, .right = 0x1357, .high = 0, .low = 0, .flags = 0xd4},
    {.left = 0x7fff, .right = 0x7fff, .high = 0x3fff, .low = 1, .flags = 0xd0},
}};

std::array<std::uint8_t, 32768> program(const Product input) {
  std::array<std::uint8_t, 32768> bytes{};
  // LDY #right; LDD #left; EMULS; STY $0062; LDY #0; LDD $0062; STD $1200.
  constexpr std::uint8_t high_shift = 8;
  const std::array<std::uint8_t, program_size> code{
      0xcd,
      static_cast<std::uint8_t>(input.right >> high_shift),
      static_cast<std::uint8_t>(input.right & byte_mask),
      0xcc,
      static_cast<std::uint8_t>(input.left >> high_shift),
      static_cast<std::uint8_t>(input.left & byte_mask),
      0x18,
      0x13,
      0x7d,
      0,
      pulse_count,
      0xcd,
      0,
      0,
      0xfc,
      0,
      pulse_count,
      0x7c,
      result_address >> high_shift,
      result_address & byte_mask};
  std::ranges::copy(code, bytes.begin());
  bytes[reset_vector - entry] = entry >> high_shift;
  return bytes;
}

void native_scratch(const Product input) {
  blofeld::McuBus board(program(input));
  board.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  std::vector<blofeld::Hc12BusCycle> operands;
  blofeld::Hc12CycleBus bus(board, [&board, &operands](const auto cycle) {
    board.advance_timer(1);
    if (cycle.address == pulse_count) { operands.push_back(cycle); }
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  board.write(timer_control, timer_fast_clear);
  for (unsigned instruction = 0; instruction < 3; ++instruction) { cpu.step(); }
  const auto product = cpu.state();
  require(product.y == input.high && product.d == input.low && product.ccr == input.flags,
          "Native signed multiply must produce the independent 32bit product and flags");
  cpu.step();
  require(board.read_memory({.address = pulse_count, .size = 2}) == input.high,
          "Native STY stores the high product in the real disabled PACNT registers");
  auto saved = board.clone();
  saved->write(pulse_count, 0);
  saved->write(pulse_count + 1, 0);
  require(board.read_memory({.address = pulse_count, .size = 2}) == input.high,
          "A copied peripheral has independent PACNT ownership");
  saved->reset();
  require(saved->read_memory({.address = pulse_count, .size = 2}) == 0,
          "Actual MCU bus reset clears its copied pulse accumulator");
  for (unsigned instruction = 0; instruction < 3; ++instruction) { cpu.step(); }
  require(cpu.state().y == 0 && cpu.state().d == input.high &&
              cpu.state().pc == entry + program_size &&
              board.read_memory({.address = result_address, .size = 2}) == input.high,
          "Native LDD retrieves PACNT independently of Y and stores the literal high product");
  require(operands.size() == 2 && operands[0].kind == 'W' && operands[1].kind == 'R',
          "Native PACNT access has one word write and one word read bus slot");
  for (const auto operand : operands) {
    require(operand.size == 2 && operand.word_part == blofeld::Hc12WordPart::whole,
            "The native aligned PACNT operand remains a whole word within one bus clock");
  }
  require(board.read(pulse_control) == 0 && board.read(pulse_control + 1) == 0,
          "Software scratch access cannot enable counting or fabricate a pulse interrupt");
}

void unrelated_flags() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus board(flash);
  constexpr unsigned timer_io = 0x40;
  constexpr unsigned channel_zero = 0x50;
  board.write(timer_io, 1);
  board.write(channel_zero, 0);
  board.write(channel_zero + 1, 1);
  board.write(timer_control, timer_on);
  board.advance_timer(word_cycles);
  require(board.read(timer_flags) == timer_on && board.read(channel_flags) == 1,
          "A real overflow and output compare establish independent timer flags");
  board.write(timer_control, timer_fast_clear);
  board.write(pulse_count, byte_mask);
  board.write(pulse_count + 1, byte_mask);
  static_cast<void>(board.read(pulse_count));
  static_cast<void>(board.read(pulse_count + 1));
  require(
      board.read(timer_flags) == timer_on && board.read(channel_flags) == 1 &&
          board.read(pulse_control + 1) == 0,
      "PACNT fast clearing cannot clear TCNT/channel flags or turn software FFFF into overflow");
}
}

int main() {
  try {
    for (const auto product : products) { native_scratch(product); }
    unrelated_flags();
    std::cout
        << "Native EMULS/STY/LDD PACNT scratch, reset/copy, word slots and flag isolation passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
