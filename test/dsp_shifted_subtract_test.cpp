// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test {
namespace {
constexpr std::uint64_t full_range = std::uint64_t{1} << 56;
constexpr auto signed_bound = static_cast<std::int64_t>(sign_bit);
constexpr unsigned input_control = 0x123456;
constexpr unsigned input_source = 0x654321;
constexpr unsigned changed_flags = carry | overflow | zero | negative | unnormalized | extension;
constexpr std::array values{std::uint64_t{0},
                            std::uint64_t{1},
                            std::uint64_t{2},
                            accumulator_mask,
                            accumulator_mask - 1,
                            sign_bit,
                            sign_bit + 1,
                            sign_bit - 1,
                            sign_bit - 2,
                            (sign_bit / 2) - 1,
                            sign_bit / 2,
                            accumulator_mask - (sign_bit / 2) + 1};
constexpr std::array scales{0U, scale_down, scale_up};
constexpr std::array statuses{0U, carry | overflow, limit | sticky, operands::all_ccr};

struct Input {
  std::uint64_t source = 0;
  std::uint64_t destination = 0;
  unsigned status = 0;
  bool left = false;
  bool destination_b = false;
};

struct Result {
  std::uint64_t value = 0;
  unsigned flags = 0;
  bool qualified_carry = false;
};

struct ParallelEffect {
  bool active = false;
  unsigned word = 0;
  unsigned flags = 0;
};

std::int64_t signed_value(const std::uint64_t value) {
  return (value & sign_bit) != 0 ? -static_cast<std::int64_t>(full_range - value)
                                 : static_cast<std::int64_t>(value);
}

bool outside_accumulator(const std::int64_t value) {
  return value < -signed_bound || value >= signed_bound;
}

// Signed host integers have room for every mathematical 2*D-S (up to 58 bits).
// This deliberately uses integer ranges/division, not native sign-bit formulas.
Result expected(const Input &input) {
  const auto destination = signed_value(input.destination);
  auto shifted = destination / 2;
  if (destination < 0 && destination % 2 != 0) { --shifted; }
  if (input.left) { shifted = destination * 2; }
  const auto mathematical = shifted - signed_value(input.source);
  const auto value = static_cast<std::uint64_t>(mathematical) & accumulator_mask;
  const bool shifted_overflow = outside_accumulator(shifted);
  unsigned flags = value_flags({.value = value, .status = input.status});
  if (shifted_overflow || outside_accumulator(mathematical)) { flags |= overflow | limit; }
  if ((static_cast<std::uint64_t>(shifted) & accumulator_mask) < input.source) { flags |= carry; }
  // FM13-174 only qualifies C when the left shift did not overflow.
  return {.value = value, .flags = flags, .qualified_carry = !shifted_overflow};
}

void seed(DspProgram &code, const Input &input) {
  operands::seed_operands(code,
                          {.a = input.destination_b ? input.source : input.destination,
                           .b = input.destination_b ? input.destination : input.source,
                           .control = input_control,
                           .source = input_source,
                           .status = input.status});
}

std::string instruction(const Input &input) {
  return std::string(input.left ? "subl " : "subr ") + (input.destination_b ? "a,b" : "b,a");
}

void verify(DspMachine &machine, const Input &input, const ParallelEffect effect = {}) {
  const auto before = machine.cpu().debug_state();
  require(before.cpu.a == (input.destination_b ? input.source : input.destination) &&
              before.cpu.b == (input.destination_b ? input.destination : input.source) &&
              before.cpu.sr == (masked_status | input.status),
          "Actual native setup supplies the independently specified accumulators and CCR");
  const auto wanted = expected(input);
  auto cpu = before.cpu;
  if (input.destination_b) {
    cpu.b = wanted.value;
  } else {
    cpu.a = wanted.value;
  }
  ++cpu.pc;
  cpu.sr = (cpu.sr & ~changed_flags) | wanted.flags | effect.flags;
  if (effect.active) { cpu.x = (cpu.x & ~std::uint64_t{word_mask}) | effect.word; }
  machine.cpu().step();
  auto after = machine.cpu().debug_state();
  const auto observed_status = after.cpu.sr;
  if (!wanted.qualified_carry) {
    // Observe but do not invent an independent silicon oracle for this C bit.
    after.cpu.sr &= ~carry;
    cpu.sr &= ~carry;
  }
  const auto context = instruction(input) + " D=" + number(input.destination) +
                       " S=" + number(input.source) + " input SR=" + number(input.status) +
                       " expected SR=" + number(cpu.sr) + " observed SR=" + number(observed_status);
  require(after.cpu == cpu && after.native_instructions == before.native_instructions + 1 &&
              after.stack_high == before.stack_high && after.stack_low == before.stack_low &&
              after.ep == before.ep && after.sz == before.sz && after.sc == before.sc,
          "Independent 56-bit mathematical subtraction and every qualified state/flag: " + context);
}

std::vector<Input> inputs(const bool left, const unsigned scale) {
  std::vector<Input> result;
  for (const auto destination : values) {
    for (const auto source : values) {
      for (const auto status : statuses) {
        for (const bool destination_b : {false, true}) {
          result.push_back({.source = source,
                            .destination = destination,
                            .status = scale | status,
                            .left = left,
                            .destination_b = destination_b});
        }
      }
    }
  }
  return result;
}

void matrix(const bool left) {
  unsigned count = 0;
  unsigned unqualified_carry = 0;
  for (const auto scale : scales) {
    const auto cases = inputs(left, scale);
    DspProgram code;
    for (const auto &input : cases) {
      seed(code, input);
      code.emit(instruction(input));
    }
    DspMachine machine;
    code.install(machine.bus());
    for (const auto &input : cases) {
      for (unsigned index = 0; index < operands::operand_preamble_steps; ++index) {
        machine.cpu().step();
      }
      verify(machine, input);
      ++count;
      if (!expected(input).qualified_carry) { ++unqualified_carry; }
    }
  }
  std::cout << (left ? "SUBL" : "SUBR") << ": " << count
            << " independent normal-mode cases passed; C has no asserted silicon oracle in "
            << unqualified_carry << " left-shift-overflow cases.\n"
            << std::flush;
}

void host_overflow() {
  // A legal SUBR result needs the full 56-bit range, not padded signed-host subtraction.
  const Input input{.source = sign_bit};
  DspProgram code;
  seed(code, input);
  code.emit(instruction(input));
  DspMachine machine;
  code.install(machine.bus());
  for (unsigned index = 0; index < operands::operand_preamble_steps; ++index) {
    machine.cpu().step();
  }
  verify(machine, input);
}

void parallel(const bool left) {
  struct Case {
    std::uint64_t source = 0;
    std::uint64_t destination = 0;
    bool destination_b = false;
    bool transfer_b = false;
    ParallelEffect effect;
  };

  constexpr std::uint64_t growth = std::uint64_t{1} << 46;
  constexpr unsigned positive_limit = 0x7fffff;
  constexpr unsigned negative_limit = 0x800000;
  constexpr unsigned growth_word = 0x400000;
  constexpr std::array cases{
      Case{.source = 1, .effect = {.active = true}},
      Case{.destination = sign_bit - 1,
           .destination_b = true,
           .transfer_b = true,
           .effect = {.active = true, .word = positive_limit, .flags = limit}},
      Case{.destination = growth, .effect = {.active = true, .word = growth_word, .flags = sticky}},
      Case{.source = growth,
           .destination_b = true,
           .effect = {.active = true, .word = growth_word, .flags = sticky}},
      Case{.source = sign_bit,
           .destination_b = true,
           .effect = {.active = true, .word = negative_limit, .flags = limit}}};
  for (const auto &item : cases) {
    const Input input{.source = item.source,
                      .destination = item.destination,
                      .status = carry | overflow,
                      .left = left,
                      .destination_b = item.destination_b};
    DspProgram code;
    seed(code, input);
    code.emit(instruction(input) + (item.transfer_b ? " b,x0" : " a,x0"));
    DspMachine machine;
    code.install(machine.bus());
    for (unsigned index = 0; index < operands::operand_preamble_steps; ++index) {
      machine.cpu().step();
    }
    verify(machine, input, item.effect);
  }
  std::cout << "Five literal parallel pre-read/growth/limiting cases passed.\n";
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 2,
                           "Usage: dsp-shifted-subtract-test --left|--right|--host-overflow");
    const std::string_view option(arguments[1]);
    if (option == "--host-overflow") {
      blofeld::test::host_overflow();
      std::cout << "Literal legal SUBR host-overflow regression passed.\n";
      return 0;
    }
    blofeld::test::require(option == "--left" || option == "--right",
                           "Expected an explicit shifted-subtraction direction");
    blofeld::test::matrix(option == "--left");
    blofeld::test::parallel(option == "--left");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
