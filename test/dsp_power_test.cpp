// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_saturation_fixture.hpp"
#include "native_access_fixture.hpp"

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
namespace arithmetic = saturation;
constexpr unsigned setup_steps = 11;
constexpr unsigned highest_tabulated_coefficient = 22;
constexpr std::size_t batch_limit = 1800;
// Five-bit native reference form: 00000001 000 sssss 11 QQ d k op.
// FM13-99/103/138/142 have inconsistent field widths; QQ=Y1/X0/Y0/X1.
constexpr unsigned product_word = 0x0100c0;
constexpr std::array<std::string_view, 4> registers{"y1", "x0", "y0", "x1"};
constexpr std::array register_seeds{0x654321U, 0x123456U, 0xabcdefU, 0x246813U};
constexpr std::array source_values{0U, 1U, 0x7fffffU, 0x800000U, 0xffffffU, 3U, 5U, 0xfffffdU};
constexpr std::array destination_values{std::uint64_t{0},
                                        accumulator_mask,
                                        sign_bit - 1,
                                        sign_bit,
                                        std::uint64_t{0x007fffffffffff},
                                        std::uint64_t{0xff800000000000},
                                        std::uint64_t{0x007fffff800000},
                                        std::uint64_t{0xff7fffff800000}};
constexpr std::array flags{0U, carry | overflow, sticky | limit, 0xffU};

struct Input {
  arithmetic::Input arithmetic;
  unsigned source_field = 0;
};

unsigned operation_bits(const arithmetic::Operation operation) {
  if (operation == arithmetic::Operation::multiply) { return 0; }
  if (operation == arithmetic::Operation::multiply_round) { return 1; }
  if (operation == arithmetic::Operation::accumulate) { return 2; }
  if (operation == arithmetic::Operation::accumulate_round) { return 3; }
  throw Fault("Expected a power-coefficient product operation");
}

std::string operation_name(const arithmetic::Operation operation) {
  constexpr std::array<std::string_view, 4> names{"MPY", "MPYR", "MAC", "MACR"};
  return std::string(names.at(operation_bits(operation)));
}

unsigned opcode(const Input &input) {
  const auto &value = input.arithmetic;
  return product_word | (value.power << 8) | (input.source_field << 4) |
         (value.destination_b ? 8U : 0U) | (value.negative_product ? 4U : 0U) |
         operation_bits(value.operation);
}

std::array<unsigned, 4> source_registers(const Input &input) {
  auto result = register_seeds;
  result.at(input.source_field) = input.arithmetic.x;
  return result;
}

void accumulator(DspProgram &code, const std::string_view name, const std::uint64_t value) {
  code.immediate(std::string(name) + '0', value & word_mask);
  code.immediate(std::string(name) + '1', (value >> word_bits) & word_mask);
  code.immediate(std::string(name) + '2', value >> data_bits);
}

void emit(DspProgram &code, const Input &input) {
  const auto &value = input.arithmetic;
  accumulator(code, "a", value.destination_b ? other_accumulator : value.destination);
  accumulator(code, "b", value.destination_b ? value.destination : other_accumulator);
  const auto words = source_registers(input);
  for (std::size_t index = 0; index < words.size(); ++index) {
    code.immediate(std::string(registers.at(index)), words.at(index));
  }
  code.immediate("sr", masked_status | value.status);
  code.word(opcode(input));
}

void check_setup(const DspDebugState &before, const Input &input) {
  const auto &value = input.arithmetic;
  const auto words = source_registers(input);
  require(before.cpu.a == (value.destination_b ? other_accumulator : value.destination) &&
              before.cpu.b == (value.destination_b ? value.destination : other_accumulator) &&
              before.cpu.x == ((std::uint64_t{words.at(3)} << word_bits) | words.at(1)) &&
              before.cpu.y == ((std::uint64_t{words.at(0)} << word_bits) | words.at(2)) &&
              before.cpu.sr == (masked_status | value.status),
          "Native setup supplies distinct product sources, both accumulators and all mode bits");
}

bool unchanged_context(const DspDebugState &before, const DspDebugState &after) {
  return before.stack_high == after.stack_high && before.stack_low == after.stack_low &&
         before.ep == after.ep && before.sz == after.sz && before.sc == after.sc;
}

void verify(DspMachine &machine, const Input &input) {
  const auto before = machine.cpu().debug_state();
  check_setup(before, input);
  const auto &value = input.arithmetic;
  const auto result = arithmetic::expected(value);
  auto expected = before.cpu;
  if (value.destination_b) {
    expected.b = result.value;
  } else {
    expected.a = result.value;
  }
  ++expected.pc;
  expected.sr = (expected.sr & ~result.flag_mask) | result.flags;
  machine.cpu().step();
  const auto after = machine.cpu().debug_state();
  const auto context = operation_name(value.operation) + " n=" + std::to_string(value.power) +
                       " source=" + std::string(registers.at(input.source_field)) +
                       " word=" + number(value.x) + " D=" + number(value.destination) +
                       " incoming SR=" + number(value.status);
  require(after.cpu == expected && unchanged_context(before, after) &&
              after.native_instructions == before.native_instructions + 1,
          "Independent fractional product, rounding, saturation, flags and whole-state result: " +
              context + " expected D=" + number(result.value) +
              " actual D=" + number(value.destination_b ? after.cpu.b : after.cpu.a) +
              " expected SR=" + number(expected.sr) + " actual SR=" + number(after.cpu.sr));
}

void variants(std::vector<Input> &result, Input input) {
  for (const auto scale : {0U, scale_down, scale_up}) {
    for (const auto mode : {0U, arithmetic::saturation_mode}) {
      for (const auto rounding : {0U, arithmetic::rounding_mode}) {
        for (const bool destination_b : {false, true}) {
          for (const bool negate : {false, true}) {
            input.arithmetic.status =
                scale | mode | rounding | flags.at(input.arithmetic.power % 4);
            input.arithmetic.destination_b = destination_b;
            input.arithmetic.negative_product = negate;
            result.push_back(input);
          }
        }
      }
    }
  }
}

std::vector<Input> inputs(const arithmetic::Operation operation) {
  std::vector<Input> result;
  // The independent oracle expresses the positive signed fraction 2^-n, not
  // the SDK decoder. FM table12-16 explicitly lists exactly n=1..22.
  for (unsigned power = 1; power <= highest_tabulated_coefficient; ++power) {
    for (unsigned source = 0; source < registers.size(); ++source) {
      for (std::size_t index = 0; index < source_values.size(); ++index) {
        variants(result,
                 {.arithmetic = {.operation = operation,
                                 .destination = destination_values.at(index),
                                 .source = other_accumulator,
                                 .x = source_values.at(index),
                                 .power = power,
                                 .form = arithmetic::ProductForm::power},
                  .source_field = source});
      }
    }
  }
  return result;
}

void run(const std::span<const Input> cases) {
  DspMachine machine;
  for (std::size_t start = 0; start < cases.size(); start += batch_limit) {
    const auto batch = cases.subspan(start, std::min(batch_limit, cases.size() - start));
    DspProgram code;
    for (const auto &input : batch) { emit(code, input); }
    machine.cpu().reset_spi_boot();
    code.install(machine.bus());
    for (const auto &input : batch) {
      for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
      verify(machine, input);
    }
  }
  std::cout << cases.size() << " fixed documented power-coefficient cases passed.\n";
}

void prepare_rejection(DspMachine &machine, const Input &input) {
  machine.cpu().reset_spi_boot();
  DspProgram code;
  emit(code, input);
  code.install(machine.bus());
  for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
}

void reject(DspMachine &machine, const Input &input) {
  const auto before = machine.cpu().debug_state();
  check_setup(before, input);
  const auto error = native_access::fault([&] { machine.cpu().step(); });
  const auto after = machine.cpu().debug_state();
  auto expected = before.cpu;
  ++expected.pc;
  require(
      error.contains("Unqualified DSP reference " + operation_name(input.arithmetic.operation)) &&
          error.contains("coefficient=" + std::to_string(input.arithmetic.power)) &&
          error.contains("pc=" + hex(before.cpu.pc, 6)) && error.contains("omr=0x000005") &&
          after.cpu == expected && unchanged_context(before, after) &&
          after.native_instructions == before.native_instructions,
      "Untabulated coefficient fails contextually before data, flags or retirement change");
  static_cast<void>(native_access::fault([&] { machine.cpu().step(); }));
  static_cast<void>(native_access::fault([&] { static_cast<void>(machine.checkpoint()); }));
}

void recovery(const Input &input) {
  DspMachine machine;
  prepare_rejection(machine, input);
  const auto before = machine.cpu().debug_state();
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    reject(machine, input);
    machine.restore(saved);
    const std::array nop{0U};
    machine.bus().install_rom({.start = before.cpu.pc, .words = nop});
    machine.cpu().step();
    auto expected = before.cpu;
    ++expected.pc;
    const auto after = machine.cpu().debug_state();
    require(after.cpu == expected && unchanged_context(before, after) &&
                after.native_instructions == before.native_instructions + 1,
            "Trusted prefetch recovery preserves all state when only the own opcode changes");
  }
}

void rejection_checks(const arithmetic::Operation operation, const bool destination_b) {
  DspMachine machine;
  unsigned checked = 0;
  for (const auto power : {0U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U}) {
    for (unsigned source = 0; source < registers.size(); ++source) {
      for (const bool negate : {false, true}) {
        const Input input{.arithmetic = {.operation = operation,
                                         .destination = sign_bit,
                                         .x = source_values.at(source + (negate ? 4U : 0U)),
                                         .status = flags.at(source),
                                         .power = power,
                                         .destination_b = destination_b,
                                         .negative_product = negate,
                                         .form = arithmetic::ProductForm::power},
                          .source_field = source};
        prepare_rejection(machine, input);
        reject(machine, input);
        ++checked;
      }
    }
  }
  if (!destination_b) {
    for (const auto power : {0U, 31U}) {
      recovery({.arithmetic = {.operation = operation, .power = power}});
    }
  }
  std::cout << checked << " untabulated coefficient rejections and " << (destination_b ? 0 : 2)
            << " recovery cases passed.\n";
}

arithmetic::Operation operation(const std::string_view scope) {
  if (scope == "mpy") { return arithmetic::Operation::multiply; }
  if (scope == "mpyr") { return arithmetic::Operation::multiply_round; }
  if (scope == "mac") { return arithmetic::Operation::accumulate; }
  if (scope == "macr") { return arithmetic::Operation::accumulate_round; }
  throw Fault("Unknown own power-coefficient diagnostic scope");
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 2,
                           "Usage: dsp-power-test [reject-a-|reject-b-]mpy|mpyr|mac|macr");
    const std::string_view scope(arguments[1]);
    if (scope.starts_with("reject-a-") || scope.starts_with("reject-b-")) {
      constexpr unsigned prefix_bytes = 9;
      constexpr unsigned accumulator_index = 7;
      blofeld::test::rejection_checks(blofeld::test::operation(scope.substr(prefix_bytes)),
                                      scope.at(accumulator_index) == 'b');
    } else {
      const auto cases = blofeld::test::inputs(blofeld::test::operation(scope));
      blofeld::test::run(cases);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
