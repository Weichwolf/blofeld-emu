// SPDX-License-Identifier: GPL-3.0-only
#include "hc12_decimal.hpp"

#include "blofeld/bus.hpp"

#include <array>
#include <cstdint>

namespace blofeld::detail {
namespace {
struct DecimalRule {
  bool carry;
  unsigned high_begin;
  unsigned high_end;
  bool half;
  unsigned low_begin;
  unsigned low_end;
  unsigned correction;
  bool corrected_carry;
};

// S12CPUV2 DAA: legal post-ABA/ADD/ADC input combinations, table on page 161.
constexpr std::array<DecimalRule, 9> decimal_rules{{
    {.carry = false,
     .high_begin = 0,
     .high_end = 9,
     .half = false,
     .low_begin = 0,
     .low_end = 9,
     .correction = 0,
     .corrected_carry = false},
    {.carry = false,
     .high_begin = 0,
     .high_end = 8,
     .half = false,
     .low_begin = 10,
     .low_end = 15,
     .correction = 0x06,
     .corrected_carry = false},
    {.carry = false,
     .high_begin = 0,
     .high_end = 9,
     .half = true,
     .low_begin = 0,
     .low_end = 3,
     .correction = 0x06,
     .corrected_carry = false},
    {.carry = false,
     .high_begin = 10,
     .high_end = 15,
     .half = false,
     .low_begin = 0,
     .low_end = 9,
     .correction = 0x60,
     .corrected_carry = true},
    {.carry = false,
     .high_begin = 9,
     .high_end = 15,
     .half = false,
     .low_begin = 10,
     .low_end = 15,
     .correction = 0x66,
     .corrected_carry = true},
    {.carry = false,
     .high_begin = 10,
     .high_end = 15,
     .half = true,
     .low_begin = 0,
     .low_end = 3,
     .correction = 0x66,
     .corrected_carry = true},
    {.carry = true,
     .high_begin = 0,
     .high_end = 2,
     .half = false,
     .low_begin = 0,
     .low_end = 9,
     .correction = 0x60,
     .corrected_carry = true},
    {.carry = true,
     .high_begin = 0,
     .high_end = 2,
     .half = false,
     .low_begin = 10,
     .low_end = 15,
     .correction = 0x66,
     .corrected_carry = true},
    {.carry = true,
     .high_begin = 0,
     .high_end = 3,
     .half = true,
     .low_begin = 0,
     .low_end = 3,
     .correction = 0x66,
     .corrected_carry = true},
}};
}

Hc12DecimalResult hc12_decimal_adjust(const Hc12DecimalInput input) {
  constexpr unsigned nibble_mask = 15;
  constexpr unsigned byte_mask = 0xff;
  const unsigned high = input.a >> 4U;
  const unsigned low = input.a & nibble_mask;
  for (const auto &rule : decimal_rules) {
    if (rule.carry == input.carry && rule.half == input.half && high >= rule.high_begin &&
        high <= rule.high_end && low >= rule.low_begin && low <= rule.low_end) {
      return {.a = static_cast<std::uint8_t>((input.a + rule.correction) & byte_mask),
              .carry = rule.corrected_carry};
    }
  }
  throw Fault("Unqualified DAA operand combination A=" + hex(input.a, 2) +
              " C=" + (input.carry ? "1" : "0") + " H=" + (input.half ? "1" : "0"));
}
}
