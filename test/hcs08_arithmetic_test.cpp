// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned stack = 0x47c, h_seed = 0x3400;
constexpr unsigned addition_flags = flag_v | flag_h | flag_n | flag_z | flag_c;
constexpr unsigned subtraction_flags = flag_v | flag_n | flag_z | flag_c;
constexpr unsigned sign_bit = 0x80;
constexpr int byte_minimum = -128, byte_maximum = 127;
constexpr int word_minimum = -32768, word_maximum = 32767;

struct Operation {
  std::uint8_t opcode = 0;
  bool add = false;
  bool carry = false;
  bool compare = false;
};

constexpr std::array operations{Operation{.opcode = 0xbb, .add = true},
                                Operation{.opcode = 0xb9, .add = true, .carry = true},
                                Operation{.opcode = 0xb0},
                                Operation{.opcode = 0xb2, .carry = true},
                                Operation{.opcode = 0xb1, .compare = true},
                                Operation{.opcode = 0xb3, .compare = true}};

int signed_byte(const unsigned value) {
  return value < sign_bit ? static_cast<int>(value)
                          : static_cast<int>(value) - static_cast<int>(byte_values);
}

struct Inputs {
  unsigned accumulator;
  unsigned memory;
  unsigned seed;
};

unsigned flags(const Operation operation, const Inputs input) {
  const auto a = input.accumulator;
  const auto b = input.memory;
  const auto seed = input.seed;
  const auto carry = operation.carry ? seed & flag_c : 0;
  const int signed_result = operation.add
                                ? signed_byte(a) + signed_byte(b) + static_cast<int>(carry)
                                : signed_byte(a) - signed_byte(b) - static_cast<int>(carry);
  const auto wide = operation.add ? a + b + carry : a + byte_values - b - carry;
  const auto result = wide & byte_mask;
  unsigned actual = seed & ~(operation.add ? addition_flags : subtraction_flags);
  if (signed_result < byte_minimum || signed_result > byte_maximum) { actual |= flag_v; }
  if (operation.add && (a & 15U) + (b & 15U) + carry > 15U) { actual |= flag_h; }
  if ((result & sign_bit) != 0) { actual |= flag_n; }
  if (result == 0) { actual |= flag_z; }
  if (operation.add ? wide >= byte_values : a < b + carry) { actual |= flag_c; }
  return actual;
}

void byte_case(Memory &bus, Hcs08 &cpu, const Operation operation, const Inputs input) {
  const auto a = input.accumulator;
  const auto b = input.memory;
  const auto seed = input.seed;
  bus.bytes[operand] = static_cast<std::uint8_t>(b);
  bus.accesses.clear();
  cpu.set_state({.pc = entry,
                 .hx = static_cast<std::uint16_t>(h_seed + a),
                 .sp = stack,
                 .a = static_cast<std::uint8_t>(a),
                 .ccr = static_cast<std::uint8_t>(seed)});
  cpu.step();
  const auto carry = operation.carry ? seed & flag_c : 0;
  const auto calculated = operation.add ? a + b + carry : a + byte_values - b - carry;
  const auto expected_a = operation.compare ? a : calculated & byte_mask;
  const Hcs08State expected{.pc = entry + 2,
                            .hx = static_cast<std::uint16_t>(h_seed + a),
                            .sp = stack,
                            .a = static_cast<std::uint8_t>(expected_a),
                            .ccr = static_cast<std::uint8_t>(flags(operation, input))};
  require(cpu.state() == expected &&
              bus.accesses == std::vector<Access>{{.address = operand, .value = b, .write = false}},
          "HCS08 arithmetic op=" + hex(operation.opcode) + " a=" + hex(a) + " b=" + hex(b) +
              " seed=" + hex(seed) + " flags=" + hex(cpu.state().ccr) +
              " expected=" + hex(expected.ccr));
}

void exhaust_bytes() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  bus.known.set(operand);
  for (const auto operation : operations) {
    const std::array code{operation.opcode, static_cast<std::uint8_t>(operand)};
    bus.install(entry, code);
    for (const unsigned seed : {flags_fixed + flag_i, byte_mask}) {
      for (unsigned a = 0; a < byte_values; ++a) {
        for (unsigned b = 0; b < byte_values; ++b) {
          byte_case(bus, cpu, operation, {.accumulator = a, .memory = b, .seed = seed});
        }
      }
    }
  }
}

void direct_word_compare() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 2> code{0x75, operand}; // Own CPHX direct.
  bus.install(entry, code);
  constexpr std::array corners{0U, 1U, 127U, 128U, 255U, 256U, 0x7fffU, 0x8000U, 0xffffU};
  bus.known.set(operand).set(operand + 1);
  for (const auto a : corners) {
    for (const auto b : corners) {
      bus.bytes[operand] = static_cast<std::uint8_t>(b >> 8U);
      bus.bytes[operand + 1] = static_cast<std::uint8_t>(b & byte_mask);
      bus.accesses.clear();
      cpu.set_state({.pc = entry,
                     .hx = static_cast<std::uint16_t>(a),
                     .sp = stack,
                     .ccr = flags_fixed + flag_i + flag_h});
      cpu.step();
      const auto result = (a + space_bytes - b) & word_mask;
      unsigned expected = flags_fixed + flag_i + flag_h;
      if (result == 0) { expected |= flag_z; }
      if ((result & 0x8000U) != 0) { expected |= flag_n; }
      if (a < b) { expected |= flag_c; }
      const auto signed_a =
          a < 0x8000U ? static_cast<int>(a) : static_cast<int>(a) - static_cast<int>(space_bytes);
      const auto signed_b =
          b < 0x8000U ? static_cast<int>(b) : static_cast<int>(b) - static_cast<int>(space_bytes);
      if (signed_a - signed_b < word_minimum || signed_a - signed_b > word_maximum) {
        expected |= flag_v;
      }
      require(cpu.state().hx == a && cpu.state().ccr == expected &&
                  bus.accesses ==
                      std::vector<Access>{
                          {.address = operand, .value = b >> 8U, .write = false},
                          {.address = operand + 1, .value = b & byte_mask, .write = false}},
              "CPHX direct must compare one big-endian operand, not dereference its value");
    }
  }
}
} // namespace

int main() {
  try {
    exhaust_bytes();
    direct_word_compare();
    std::cout << "786432 independent HCS08 byte arithmetic and81 direct-word cases passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
