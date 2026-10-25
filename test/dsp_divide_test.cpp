// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_instruction_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr std::uint64_t accumulator_range = std::uint64_t{1} << 56;
constexpr std::int64_t source_range = std::int64_t{1} << word_bits;
constexpr std::int64_t source_sign = source_range / 2;
constexpr unsigned divide_word = 0x018040; // FM13-54: DIV X0,A; JJ at4, D at3.
constexpr unsigned setup_steps = 11;
constexpr unsigned arithmetic_16 = 0x020000;
constexpr std::size_t batch_size = 256;
constexpr unsigned quotient_iterations = 24;
constexpr std::array<std::string_view, 4> names{"x0", "y0", "x1", "y1"};
constexpr std::array source_seeds{0x13579aU, 0x2468abU, 0x3579bcU, 0x468acdU};

struct Input {
  std::uint64_t value = 0;
  unsigned divisor = 0;
  unsigned field = 0;
  unsigned status = 0;
  bool destination_b = false;
};

struct Result {
  std::uint64_t value = 0;
  unsigned status = 0;
};

Result iteration(const Input &input) {
  // Independent right-aligned integer model of FM13-53's three operations.
  // Even the largest intermediate magnitude is below 2^56, within signed64.
  const auto shifted = ((input.value * 2) + (input.status & carry)) & accumulator_mask;
  const auto signed_shifted = (shifted & sign_bit) != 0
                                  ? -static_cast<std::int64_t>(accumulator_range - shifted)
                                  : static_cast<std::int64_t>(shifted);
  auto divisor = static_cast<std::int64_t>(input.divisor);
  if (divisor >= source_sign) { divisor -= source_range; }
  divisor *= source_range;
  const bool add = ((input.value & sign_bit) != 0) != (input.divisor >= source_sign);
  const auto calculated = signed_shifted + (add ? divisor : -divisor);
  const auto value = std::bit_cast<std::uint64_t>(calculated) & accumulator_mask;
  auto status = input.status & ~(carry | overflow);
  if ((value & sign_bit) == 0) { status |= carry; }
  if (((input.value ^ shifted) & sign_bit) != 0) { status |= overflow | limit; }
  return {.value = value, .status = status};
}

void accumulator(DspProgram &code, const std::string_view name, const std::uint64_t value) {
  code.immediate(std::string(name) + '0', value & word_mask);
  code.immediate(std::string(name) + '1', (value >> word_bits) & word_mask);
  code.immediate(std::string(name) + '2', value >> data_bits);
}

std::array<unsigned, 4> sources(const Input &input) {
  auto result = source_seeds;
  result.at(input.field) = input.divisor;
  return result;
}

void seed(DspProgram &code, const Input &input) {
  accumulator(code, "a", input.destination_b ? other_accumulator : input.value);
  accumulator(code, "b", input.destination_b ? input.value : other_accumulator);
  const auto words = sources(input);
  for (std::size_t index = 0; index < words.size(); ++index) {
    code.immediate(std::string(names.at(index)), words.at(index));
  }
  code.immediate("sr", masked_status | input.status);
}

unsigned opcode(const Input &input) {
  return divide_word | (input.field << 4) | (input.destination_b ? 8U : 0U);
}

void verify(DspMachine &machine, const Input &input) {
  const auto before = machine.cpu().debug_state();
  const auto words = sources(input);
  require(before.cpu.a == (input.destination_b ? other_accumulator : input.value) &&
              before.cpu.b == (input.destination_b ? input.value : other_accumulator) &&
              before.cpu.x == ((std::uint64_t{words.at(2)} << word_bits) | words.at(0)) &&
              before.cpu.y == ((std::uint64_t{words.at(3)} << word_bits) | words.at(1)) &&
              before.cpu.sr == (masked_status | input.status),
          "Actual native MOVEs establish both accumulators, all sources, modes and flags");
  const auto result = iteration(input);
  auto wanted = before;
  ++wanted.cpu.pc;
  ++wanted.native_instructions;
  ++wanted.native_cycles; // FM A-1 nominal DIV cost, not a physical pipeline clock.
  wanted.cpu.sr = masked_status | result.status;
  if (input.destination_b) {
    wanted.cpu.b = result.value;
  } else {
    wanted.cpu.a = result.value;
  }
  machine.cpu().step();
  require(machine.cpu().debug_state() == wanted,
          "Independent DIV result/whole-state mismatch: D=" + number(input.value) +
              " S=" + number(input.divisor) + " JJ=" + number(input.field) +
              " incoming SR=" + number(input.status));
}

std::vector<Input> matrix() {
  constexpr std::array<std::uint64_t, 16> dividends{0,
                                                    1,
                                                    0xffffff,
                                                    0x1000000,
                                                    0x200000000000,
                                                    0x7fffffffffff,
                                                    0x800000000000,
                                                    0xff800000000000,
                                                    0xfffffffffffffe,
                                                    accumulator_mask,
                                                    sign_bit,
                                                    sign_bit + 1,
                                                    sign_bit - 1,
                                                    0x40000000000000,
                                                    0xbfffffffffffff,
                                                    other_accumulator};
  constexpr std::array divisors{
      0U, 1U, 2U, 0x3fffffU, 0x400000U, 0x7fffffU, 0x800000U, 0xa00000U, word_mask};
  constexpr unsigned all_flags = 0xff;
  constexpr unsigned preserved = sticky | extension | unnormalized | negative | zero;
  std::vector<Input> result;
  for (const auto value : dividends) {
    for (const auto divisor : divisors) {
      for (unsigned field = 0; field < names.size(); ++field) {
        for (const auto status : {0U, carry, overflow, overflow | carry, preserved, all_flags}) {
          for (const auto scale : {0U, scale_down, scale_up}) {
            for (const bool b : {false, true}) {
              result.push_back({.value = value,
                                .divisor = divisor,
                                .field = field,
                                .status = status | scale,
                                .destination_b = b});
            }
          }
        }
      }
    }
  }
  return result;
}

void independent_matrix(const std::span<const Input> inputs) {
  for (std::size_t begin = 0; begin < inputs.size(); begin += batch_size) {
    const auto cases = inputs.subspan(begin, std::min(batch_size, inputs.size() - begin));
    DspMachine machine;
    DspProgram code;
    for (const auto &input : cases) {
      seed(code, input);
      code.word(opcode(input));
    }
    code.install(machine.bus());
    for (const auto &input : cases) {
      for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
      verify(machine, input);
    }
  }
}

void preserved_overflow_input() {
  // Own 45-byte regression: fourteen 24-bit seeds, then FM's DIV X0,A.
  // The prior SDK subtracts a positive operand from host INT64_MIN here.
  constexpr std::array<std::uint8_t, 45> bytes{
      0x7f, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0,    0,    0, 0, 0, 0,    0x40, 0,   0,
      0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0, 0, 0, 0x01, 0x80, 0x40};
  const auto input = decode_instruction_input(bytes);
  if (!input) { throw Fault("The preserved DIV overflow regression must decode"); }
  const auto result = execute_instruction_input(input.value(), 1);
  constexpr std::uint64_t expected = 0x7f800001000000;
  require(result.stop == InstructionStop::budget && result.steps == 1 &&
              result.state.cpu.a == expected &&
              result.state.cpu.sr == (masked_status | carry | overflow | limit),
          "The literal host-overflow input has an independent modular result and V/L/C");
}

void repeat_step(DspMachine &machine, Input &input) {
  const auto result = iteration(input);
  machine.cpu().step();
  const auto actual = machine.cpu().state();
  require((input.destination_b ? actual.b : actual.a) == result.value &&
              actual.sr == (masked_status | result.status),
          "Each real latched DIV consumes the preceding result and carry");
  input.value = result.value;
  input.status = result.status;
}

void repeat_case(const std::uint64_t value, const bool b) {
  constexpr unsigned divisor = 0x400000;
  constexpr unsigned split = 8;
  DspMachine machine;
  DspProgram code;
  Input input{.value = value, .divisor = divisor, .destination_b = b};
  seed(code, input);
  code.emit("rep #24");
  const auto body = code.pc();
  code.word(opcode(input));
  code.emit("nop");
  code.install(machine.bus());
  for (unsigned count = 0; count <= setup_steps; ++count) { machine.cpu().step(); }
  for (unsigned count = 0; count < split; ++count) { repeat_step(machine, input); }
  // Own controlled ROM/cache mutation: the active REP still holds DIV.
  constexpr std::array<std::uint32_t, 1> replacement{0};
  machine.bus().install_rom({.start = body, .words = replacement});
  const auto saved = machine.checkpoint();
  const auto partial = input;
  DspDebugState expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    if (replay != 0) { machine.restore(saved); }
    input = partial;
    for (unsigned count = split; count < quotient_iterations; ++count) {
      repeat_step(machine, input);
    }
    const auto actual = machine.cpu().debug_state();
    require(actual.cpu.pc == body + 1 && ((b ? actual.cpu.b : actual.cpu.a) & word_mask) ==
                                             value / (std::uint64_t{2} * divisor),
            "24 iterations yield the independently calculated positive fractional quotient");
    if (replay == 0) { expected = actual; }
    require(actual == expected, "Both trusted suffixes preserve full native quotient state");
  }
}

void repeated_quotients() {
  constexpr std::array<std::uint64_t, 4> dividends{0, 1, 0x200000000000, 0x3fffffffffff};
  for (const auto value : dividends) {
    for (const bool b : {false, true}) { repeat_case(value, b); }
  }
}

void unqualified_sixteen_bit() {
  DspMachine machine;
  DspProgram code;
  const Input input{
      .value = other_accumulator, .divisor = 0x400000, .status = arithmetic_16 | carry};
  seed(code, input);
  code.word(opcode(input));
  code.install(machine.bus());
  for (unsigned count = 0; count < setup_steps; ++count) { machine.cpu().step(); }
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().state();
  try {
    machine.cpu().step();
  } catch (const Fault &error) {
    const std::string message(error.what());
    require(message.contains("DIV sixteen-bit arithmetic") && message.contains("pc=") &&
                message.contains("omr=") && machine.cpu().state().a == before.a &&
                machine.cpu().state().b == before.b,
            "Unqualified sixteen-bit division rejects before native arithmetic");
    machine.restore(saved);
    static_cast<void>(machine.checkpoint());
    return;
  }
  throw Fault("Reference DIV silently accepted unqualified sixteen-bit arithmetic");
}
}

int main() {
  try {
    preserved_overflow_input();
    const auto inputs = matrix();
    independent_matrix(inputs);
    repeated_quotients();
    unqualified_sixteen_bit();
    std::cout << inputs.size()
              << " independent DIV cases, literal overflow and REP replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
