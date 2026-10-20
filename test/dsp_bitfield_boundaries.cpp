// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace blofeld::test::operands {
namespace {
enum class Operation : unsigned char { signed_extract, unsigned_extract, insert };

struct Field {
  unsigned width = 0;
  unsigned offset = 0;
};

struct BitfieldCase {
  Operation operation = Operation::signed_extract;
  Field field;
  OperandInput input;
  bool source_b = false;
  bool destination_b = false;
  bool immediate = false;
};

constexpr std::array fields{Field{.width = 0, .offset = 0},
                            Field{.width = 0, .offset = 56},
                            Field{.width = 1, .offset = 0},
                            Field{.width = 1, .offset = 55},
                            Field{.width = 24, .offset = 0},
                            Field{.width = 24, .offset = 32},
                            Field{.width = 32, .offset = 24},
                            Field{.width = 55, .offset = 1},
                            Field{.width = 56, .offset = 0}};
constexpr std::array values{std::uint64_t{0},
                            accumulator_mask,
                            std::uint64_t{0x0123456789abcd},
                            std::uint64_t{0x80fedcba987654}};
constexpr std::array statuses{0U, carry | overflow, all_ccr};
constexpr unsigned width_shift = 12;
constexpr unsigned changed_flags = carry | overflow | zero | negative | unnormalized | extension;

std::string name(const Operation operation) {
  switch (operation) {
    case Operation::signed_extract: return "extract";
    case Operation::unsigned_extract: return "extractu";
    case Operation::insert: return "insert";
    default: throw Fault("Unknown own bitfield operation");
  }
}

BitfieldCase make_case(const Operation operation, const Field field, const unsigned mode) {
  return {.operation = operation,
          .field = field,
          .input = {},
          .source_b = (mode & 1U) != 0,
          .destination_b = (mode & 2U) != 0,
          .immediate = (mode & 4U) != 0};
}

void append_inputs(std::vector<BitfieldCase> &inputs, const BitfieldCase item) {
  for (const auto value : values) {
    for (const auto status : statuses) {
      auto next = item;
      next.input = {.a = value,
                    .b = value ^ accumulator_mask,
                    .control = (item.field.width << width_shift) | item.field.offset,
                    .source = static_cast<unsigned>(value & word_mask),
                    .status = status};
      inputs.push_back(next);
    }
  }
}

std::vector<BitfieldCase> cases() {
  std::vector<BitfieldCase> inputs;
  for (const auto operation :
       {Operation::signed_extract, Operation::unsigned_extract, Operation::insert}) {
    for (const auto field : fields) {
      if (operation == Operation::insert && field.width > word_bits) { continue; }
      for (unsigned mode = 0; mode < 8; ++mode) {
        append_inputs(inputs, make_case(operation, field, mode));
      }
    }
  }
  return inputs;
}

std::string instruction(const BitfieldCase &item) {
  const auto control = item.immediate ? '#' + number(item.input.control) : "x0";
  const char *source = item.source_b ? "b" : "a";
  if (item.operation == Operation::insert) { source = item.source_b ? "y1" : "x1"; }
  return name(item.operation) + ' ' + control + ',' + source + (item.destination_b ? ",b" : ",a");
}

std::uint64_t expected_value(const BitfieldCase &item) {
  const auto source = item.source_b ? item.input.b : item.input.a;
  std::uint64_t result = 0;
  if (item.operation == Operation::insert) {
    result = item.destination_b ? item.input.b : item.input.a;
    const auto word = item.source_b ? item.input.source ^ word_mask : item.input.source;
    for (unsigned bit = 0; bit < item.field.width; ++bit) {
      const auto position = bit + item.field.offset;
      const auto mask = std::uint64_t{1} << position;
      result = (result & ~mask) | (std::uint64_t{(word >> bit) & 1U} << position);
    }
    return result;
  }
  // FM13-70/72: independent individual bit selection and explicit sign fill.
  for (unsigned bit = 0; bit < item.field.width; ++bit) {
    result |= ((source >> (item.field.offset + bit)) & 1U) << bit;
  }
  if (item.operation == Operation::signed_extract && item.field.width != 0 &&
      ((result >> (item.field.width - 1)) & 1U) != 0) {
    constexpr unsigned accumulator_bits = 56;
    for (unsigned bit = item.field.width; bit < accumulator_bits; ++bit) {
      result |= std::uint64_t{1} << bit;
    }
  }
  return result;
}

void verify(const BitfieldCase &item, DspMachine &machine) {
  const auto before = machine.cpu().debug_state();
  require(before.cpu.a == item.input.a && before.cpu.b == item.input.b &&
              before.cpu.sr == (masked_status | item.input.status),
          "Actual bitfield preamble supplies the independent source and destination values");
  const auto value = expected_value(item);
  auto expected = before.cpu;
  if (item.destination_b) {
    expected.b = value;
  } else {
    expected.a = value;
  }
  expected.pc += item.immediate ? 2U : 1U;
  expected.sr =
      (before.cpu.sr & ~changed_flags) | value_flags({.value = value, .status = before.cpu.sr});
  machine.cpu().step();
  const auto after = machine.cpu().debug_state();
  const auto context = instruction(item) + " width=" + std::to_string(item.field.width) +
                       " offset=" + std::to_string(item.field.offset);
  require(after.cpu == expected && after.native_instructions == before.native_instructions + 1 &&
              after.stack_high == before.stack_high && after.stack_low == before.stack_low &&
              after.ep == before.ep && after.sz == before.sz && after.sc == before.sc,
          "Independent field bits, aliasing, whole-state preservation and flags: " + context);
}
}

void bitfield_checks() {
  const auto inputs = cases();
  DspProgram code;
  for (const auto &item : inputs) {
    seed_operands(code, item.input);
    code.emit(instruction(item));
  }
  DspMachine machine;
  code.install(machine.bus());
  for (const auto &item : inputs) {
    for (unsigned index = 0; index < operand_preamble_steps; ++index) { machine.cpu().step(); }
    verify(item, machine);
  }
  std::cout << inputs.size() << " legal normal-mode bitfield cases passed.\n" << std::flush;
}
}
