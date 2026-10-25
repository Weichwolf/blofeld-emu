// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test {
namespace {
constexpr std::uint64_t full_range = std::uint64_t{1} << 56;
constexpr std::int64_t signed_bound = std::int64_t{1} << 55;
constexpr std::int64_t word_bound = std::int64_t{1} << 23;
constexpr std::int64_t fraction_bound = std::int64_t{1} << 47;
constexpr unsigned compare_magnitude_word = 0x200007; // FM13-47, JJJ=0, D=A, no parallel move.
constexpr unsigned setup_steps = 11;
constexpr std::size_t batch_limit = 1800;
constexpr unsigned changed_flags = carry | overflow | negative | zero | extension | unnormalized;
constexpr std::array values{std::uint64_t{0},
                            std::uint64_t{1},
                            std::uint64_t{2},
                            accumulator_mask,
                            accumulator_mask - 1,
                            sign_bit,
                            sign_bit + 1,
                            sign_bit - 1,
                            sign_bit - 2,
                            std::uint64_t{0x007fffffffffff},
                            std::uint64_t{0x00800000000000},
                            std::uint64_t{0xff800000000000},
                            std::uint64_t{0xff7fffffffffff},
                            std::uint64_t{1} << 46,
                            accumulator_mask - (std::uint64_t{1} << 46) + 1,
                            other_accumulator};
constexpr std::array word_values{0U, 1U, 2U, 0x7fffffU, 0x800000U, 0xfffffeU, word_mask};
constexpr std::array<std::string_view, 4> registers{"x0", "y0", "x1", "y1"};
constexpr std::array register_seeds{0x246813U, 0xabcdefU, 0x135790U, 0x654321U};

struct Input {
  std::uint64_t destination = 0;
  std::uint64_t source = 0;
  unsigned source_field = 0;
  unsigned status = 0;
  bool destination_b = false;
};

struct Expected {
  unsigned status = 0;
  bool qualified_carry = false;
};

std::int64_t signed_accumulator(const std::uint64_t value) {
  return (value & sign_bit) != 0 ? -static_cast<std::int64_t>(full_range - value)
                                 : static_cast<std::int64_t>(value);
}

std::int64_t source_value(const Input &input) {
  if (input.source_field == 0) { return signed_accumulator(input.source); }
  auto word = static_cast<std::int64_t>(input.source);
  if (word >= word_bound) { word -= word_bound * 2; }
  return word * (std::int64_t{1} << word_bits);
}

Expected expected(const Input &input) {
  const auto destination = signed_accumulator(input.destination);
  const auto source = source_value(input);
  // Right-aligned mathematical magnitudes fit signed64, including |-2^55|.
  const auto left = destination < 0 ? -destination : destination;
  const auto right = source < 0 ? -source : source;
  const auto difference = left - right;
  const auto raw = static_cast<std::uint64_t>(difference) & accumulator_mask;
  unsigned status =
      (input.status & ~changed_flags) | value_flags({.value = raw, .status = input.status});
  // Table5-1's signed56 result rule gives positive overflow for |min56|-0.
  // This is a manual-derived reference expectation, not a silicon capture.
  if (difference < -signed_bound || difference >= signed_bound) { status |= overflow | limit; }
  if (left < right) { status |= carry; }
  // FM13-47 qualifies C only when S2 is properly sign-extended.
  return {.status = status,
          .qualified_carry = destination >= -fraction_bound && destination < fraction_bound};
}

std::array<unsigned, 4> source_registers(const Input &input) {
  auto result = register_seeds;
  if (input.source_field != 0) {
    result.at(input.source_field - 4) = static_cast<unsigned>(input.source);
  }
  return result;
}

void seed_accumulator(DspProgram &code, const std::string_view name, const std::uint64_t value) {
  code.immediate(std::string(name) + '0', value & word_mask);
  code.immediate(std::string(name) + '1', (value >> word_bits) & word_mask);
  code.immediate(std::string(name) + '2', value >> data_bits);
}

void emit(DspProgram &code, const Input &input) {
  const auto other = input.source_field == 0 ? input.source : other_accumulator;
  seed_accumulator(code, "a", input.destination_b ? other : input.destination);
  seed_accumulator(code, "b", input.destination_b ? input.destination : other);
  const auto words = source_registers(input);
  for (std::size_t index = 0; index < words.size(); ++index) {
    code.immediate(std::string(registers.at(index)), words.at(index));
  }
  code.immediate("sr", masked_status | input.status);
  code.word(compare_magnitude_word | (input.source_field << 4) | (input.destination_b ? 8U : 0U));
}

void verify(DspMachine &machine, const Input &input) {
  const auto before = machine.cpu().debug_state();
  const auto other = input.source_field == 0 ? input.source : other_accumulator;
  const auto words = source_registers(input);
  const auto x = (std::uint64_t{words.at(2)} << word_bits) | words.at(0);
  const auto y = (std::uint64_t{words.at(3)} << word_bits) | words.at(1);
  require(before.cpu.a == (input.destination_b ? other : input.destination) &&
              before.cpu.b == (input.destination_b ? input.destination : other) &&
              before.cpu.x == x && before.cpu.y == y &&
              before.cpu.sr == (masked_status | input.status),
          "Actual native setup supplies all specified operands, modes and flags");
  const auto wanted = expected(input);
  auto cpu = before.cpu;
  ++cpu.pc;
  cpu.sr = masked_status | wanted.status;
  machine.cpu().step();
  auto after = machine.cpu().debug_state();
  const auto observed_status = after.cpu.sr;
  if (!wanted.qualified_carry) {
    cpu.sr &= ~carry;
    after.cpu.sr &= ~carry;
  }
  const auto context = "CMPM destination=" + number(input.destination) +
                       " source=" + number(input.source) + " field=" + number(input.source_field) +
                       " incoming SR=" + number(input.status) + " expected SR=" + number(cpu.sr) +
                       " observed SR=" + number(observed_status);
  require(after.cpu == cpu && after.native_instructions == before.native_instructions + 1 &&
              after.stack_high == before.stack_high && after.stack_low == before.stack_low &&
              after.ep == before.ep && after.sz == before.sz && after.sc == before.sc,
          "Independent CMPM magnitude/result flags and unchanged operands/whole state: " + context);
}

void variants(std::vector<Input> &result, Input input) {
  for (const auto scale : {0U, scale_down, scale_up}) {
    for (const auto flags : {0U, carry | overflow, limit | sticky, 0xffU}) {
      for (const bool destination_b : {false, true}) {
        input.destination_b = destination_b;
        input.status = scale | flags;
        result.push_back(input);
      }
    }
  }
}

std::vector<Input> inputs(const std::string_view scope) {
  std::vector<Input> result;
  if (scope == "--full") {
    for (const auto destination : values) {
      for (const auto source : values) {
        variants(result, {.destination = destination, .source = source});
      }
    }
  } else if (scope == "--word") {
    for (const auto field : {4U, 5U, 6U, 7U}) {
      for (const auto destination : values) {
        for (const auto source : word_values) {
          variants(result, {.destination = destination, .source = source, .source_field = field});
        }
      }
    }
  } else if (scope == "--minimum") {
    for (const bool destination_b : {false, true}) {
      result.push_back({.destination = sign_bit, .destination_b = destination_b});
      result.push_back({.source = sign_bit, .destination_b = destination_b});
      result.push_back(
          {.destination = sign_bit, .source = sign_bit, .destination_b = destination_b});
    }
  } else {
    throw Fault("Unknown own CMPM diagnostic scope");
  }
  return result;
}

void run(const std::span<const Input> cases) {
  DspMachine machine;
  unsigned qualified = 0;
  for (std::size_t start = 0; start < cases.size(); start += batch_limit) {
    const auto batch = cases.subspan(start, std::min(batch_limit, cases.size() - start));
    DspProgram code;
    for (const auto &input : batch) { emit(code, input); }
    machine.cpu().reset_spi_boot();
    code.install(machine.bus());
    for (const auto &input : batch) {
      for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
      verify(machine, input);
      if (expected(input).qualified_carry) { ++qualified; }
    }
  }
  std::cout << cases.size() << " independent normal CMPM cases passed; C independently checked in "
            << qualified << " properly sign-extended destination cases.\n";
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 2, "Usage: dsp-cmpm-test --full|--word|--minimum");
    const auto cases = blofeld::test::inputs(std::string_view(arguments[1]));
    blofeld::test::run(cases);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
