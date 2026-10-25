// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace blofeld::test::operands {
namespace {
constexpr unsigned low_word = 0x123456;
constexpr unsigned other_word = 0x135790;
constexpr unsigned upper_a = 0x55;
constexpr unsigned upper_b = 0xaa;
constexpr unsigned preamble_instructions = 8;
constexpr unsigned changed_flags = carry | overflow | zero | negative;
constexpr std::array words{0U, 1U, 0x800001U, word_mask};
constexpr std::array statuses{0U, carry | overflow, all_ccr};

struct LogicalCase {
  bool left = false;
  bool b = false;
  bool immediate = false;
  unsigned count = 0;
  unsigned word = 0;
  unsigned status = 0;
};

struct LogicalResult {
  unsigned word = 0;
  bool last = false;
};

LogicalResult expected_word(const LogicalCase &item) {
  LogicalResult result{.word = item.word};
  for (unsigned index = 0; index < item.count; ++index) {
    if (item.left) {
      result.last = (result.word >> (word_bits - 1)) != 0;
      result.word = (result.word << 1U) & word_mask;
    } else {
      result.last = (result.word & 1U) != 0;
      result.word >>= 1U;
    }
  }
  return result;
}

void seed(DspProgram &code, const LogicalCase &item) {
  code.immediate("a0", low_word);
  code.immediate("a1", item.b ? other_word : item.word);
  code.immediate("a2", upper_a);
  code.immediate("b0", low_word);
  code.immediate("b1", item.b ? item.word : other_word);
  code.immediate("b2", upper_b);
  code.immediate("x0", item.count);
  code.immediate("sr", masked_status | item.status);
}

std::vector<LogicalCase> cases() {
  std::vector<LogicalCase> result;
  result.reserve(std::size_t{8} * (word_bits + 1) * words.size() * statuses.size());
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned count = 0; count <= word_bits; ++count) {
      for (const auto word : words) {
        for (const auto status : statuses) {
          result.push_back({.left = (mode & 1U) != 0,
                            .b = (mode & 2U) != 0,
                            .immediate = (mode & 4U) != 0,
                            .count = count,
                            .word = word,
                            .status = status});
        }
      }
    }
  }
  return result;
}

void verify(const LogicalCase &item, DspMachine &machine) {
  const auto before = machine.cpu().debug_state();
  const auto original = item.b ? before.cpu.b : before.cpu.a;
  require(((original >> word_bits) & word_mask) == item.word &&
              (original & word_mask) == low_word &&
              (original >> data_bits) == (item.b ? upper_b : upper_a) &&
              before.cpu.sr == (masked_status | item.status),
          "Actual preamble instructions supply the independent logical-shift input");
  auto expected = before.cpu;
  const auto result = expected_word(item);
  constexpr auto middle_mask = std::uint64_t{word_mask} << word_bits;
  const auto value = (original & ~middle_mask) | (std::uint64_t{result.word} << word_bits);
  if (item.b) {
    expected.b = value;
  } else {
    expected.a = value;
  }
  ++expected.pc;
  expected.sr &= ~changed_flags;
  if (result.last) { expected.sr |= carry; }
  if (result.word == 0) { expected.sr |= zero; }
  if ((result.word >> (word_bits - 1)) != 0) { expected.sr |= negative; }
  // FM3.2.5/5.4.1.2: S changes on full-accumulator bus reads, not this bare word shift.
  machine.cpu().step();
  const auto after = machine.cpu().debug_state();
  const auto context = std::string(item.left ? "LSL" : "LSR") +
                       " count=" + std::to_string(item.count) + " word=" + number(item.word) +
                       " expected SR=" + number(expected.sr) +
                       " observed SR=" + number(after.cpu.sr);
  require(after.cpu == expected && after.native_instructions == before.native_instructions + 1 &&
              after.stack_high == before.stack_high && after.stack_low == before.stack_low &&
              after.ep == before.ep && after.sz == before.sz && after.sc == before.sc,
          "FM13-93..96: repeated single-bit oracle, A2/A0 preservation and literal flags: " +
              context);
}

void run() {
  const auto inputs = cases();
  DspProgram code;
  for (const auto &item : inputs) {
    seed(code, item);
    const auto source = item.immediate ? '#' + number(item.count) : "x0";
    code.emit(std::string(item.left ? "lsl " : "lsr ") + source + (item.b ? ",b" : ",a"));
  }
  DspMachine machine;
  code.install(machine.bus());
  for (const auto &item : inputs) {
    for (unsigned index = 0; index < preamble_instructions; ++index) { machine.cpu().step(); }
    verify(item, machine);
  }
  std::cout << inputs.size()
            << " logical word cases passed: all counts 0..24, A/B, immediate/register, "
               "four input words and three complete CCR seeds.\n"
            << std::flush;
}
}

void logical_checks() {
  run();
}
}
