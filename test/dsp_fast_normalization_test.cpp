// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned clb_word = 0x0c1e00;   // FM13-43: S at1, D at0.
constexpr unsigned normf_word = 0x0c1e20; // FM13-148: sss at1, D at0.
constexpr unsigned arithmetic_16 = 0x020000;
constexpr unsigned setup_steps = 11;
constexpr unsigned accumulator_bits = 56;
constexpr unsigned accumulator_sign = accumulator_bits - 1;
constexpr std::array source_seeds{0x13579aU, 0x2468abU, 0x3579bcU, 0x468acdU};
constexpr std::size_t batch_size = 256;
constexpr std::array flags{0U, carry, overflow | limit, 0xffU};
constexpr std::array scales{0U, scale_down, scale_up};
constexpr std::array source_names{"x0", "y0", "x1", "y1"};

struct Input {
  std::uint64_t value = 0;
  int shift = 0;
  unsigned field = 0;
  unsigned status = 0;
  bool destination_b = false;
  bool count = false;
};

struct Seeds {
  std::uint64_t a = 0;
  std::uint64_t b = 0;
  std::array<unsigned, 4> words = source_seeds;
};

struct Result {
  std::uint64_t value = 0;
  unsigned status = 0;
};

unsigned shift_word(const int shift) {
  return std::bit_cast<unsigned>(shift) & word_mask;
}

Seeds seeds(const Input &input) {
  Seeds result{.a = input.destination_b ? other_accumulator : input.value,
               .b = input.destination_b ? input.value : other_accumulator};
  if (input.count) {
    result.a = input.field == 0 ? input.value : other_accumulator;
    result.b = input.field == 1 ? input.value : other_accumulator;
  } else if (input.field < 4) {
    constexpr auto msp_mask = std::uint64_t{word_mask} << word_bits;
    auto &source = input.field == 2 ? result.a : result.b;
    source = (source & ~msp_mask) | (std::uint64_t{shift_word(input.shift)} << word_bits);
  } else {
    result.words.at(input.field - 4) = shift_word(input.shift);
  }
  return result;
}

void accumulator(DspProgram &code, const std::string &name, const std::uint64_t value) {
  code.immediate(name + '0', value & word_mask);
  code.immediate(name + '1', (value >> word_bits) & word_mask);
  code.immediate(name + '2', value >> data_bits);
}

void seed(DspProgram &code, const Input &input) {
  const auto values = seeds(input);
  accumulator(code, "a", values.a);
  accumulator(code, "b", values.b);
  for (std::size_t index = 0; index < values.words.size(); ++index) {
    code.immediate(source_names.at(index), values.words.at(index));
  }
  code.immediate("sr", masked_status | input.status);
}

unsigned opcode(const Input &input) {
  return (input.count ? clb_word : normf_word) | (input.field << 1) |
         (input.destination_b ? 1U : 0U);
}

int leading_count(const std::uint64_t value) {
  if (value == 0) { return 0; }
  const bool sign = (value & sign_bit) != 0;
  int consecutive = 0;
  for (int bit = static_cast<int>(accumulator_sign); bit >= 0; --bit) {
    if (((value >> static_cast<unsigned>(bit)) & 1U) != static_cast<unsigned>(sign)) { break; }
    ++consecutive;
  }
  return 9 - consecutive;
}

Result expected(const Input &input) {
  if (input.count) {
    const auto count = leading_count(input.value);
    const auto aligned = std::int64_t{count} * (std::int64_t{1} << word_bits);
    return {.value = std::bit_cast<std::uint64_t>(aligned) & accumulator_mask,
            .status = (input.status & ~(negative | zero | overflow)) | (count < 0 ? negative : 0U) |
                      (count == 0 ? zero : 0U)};
  }
  const auto initial = seeds(input);
  auto value = input.destination_b ? initial.b : initial.a;
  bool overflowed = false;
  const auto count = input.shift < 0 ? -input.shift : input.shift;
  // An independent sequence of single-bit operations, without a wide host shift.
  for (int step = 0; step < count; ++step) {
    const auto before = value;
    value = input.shift < 0 ? (value * 2) & accumulator_mask : (value >> 1) | (value & sign_bit);
    if (input.shift < 0 && ((before ^ value) & sign_bit) != 0) { overflowed = true; }
  }
  constexpr unsigned changed = extension | unnormalized | negative | zero | overflow;
  auto status = (input.status & ~changed) | value_flags({.value = value, .status = input.status});
  if (overflowed) { status |= overflow | limit; }
  return {.value = value, .status = status};
}

void verify(DspMachine &machine, const Input &input, const bool check_seeds = true) {
  const auto before = machine.cpu().debug_state();
  if (check_seeds) {
    const auto initial = seeds(input);
    require(before.cpu.a == initial.a && before.cpu.b == initial.b &&
                before.cpu.x ==
                    ((std::uint64_t{initial.words.at(2)} << word_bits) | initial.words.at(0)) &&
                before.cpu.y ==
                    ((std::uint64_t{initial.words.at(3)} << word_bits) | initial.words.at(1)) &&
                before.cpu.sr == (masked_status | input.status),
            "Native MOVEs establish both accumulators, sources and status");
  }
  const auto result = expected(input);
  auto wanted = before;
  ++wanted.cpu.pc;
  ++wanted.native_instructions;
  ++wanted.native_cycles; // FM A-1 nominal instruction cost, not a physical pipeline clock.
  wanted.cpu.sr = masked_status | result.status;
  if (input.destination_b) {
    wanted.cpu.b = result.value;
  } else {
    wanted.cpu.a = result.value;
  }
  machine.cpu().step();
  require(machine.cpu().debug_state() == wanted,
          "Independent CLB/NORMF whole-state mismatch: value=" + number(input.value) +
              " source=" + number(input.field) + " shift=" + std::to_string(input.shift) +
              " status=" + number(input.status));
}

std::vector<std::uint64_t> count_values() {
  std::vector<std::uint64_t> result{0, accumulator_mask};
  for (unsigned bit = 0; bit < accumulator_bits; ++bit) {
    const auto value = std::uint64_t{1} << bit;
    result.push_back(value);
    result.push_back(accumulator_mask ^ value);
  }
  return result;
}

std::vector<Input> count_matrix() {
  std::vector<Input> result;
  for (const auto value : count_values()) {
    for (unsigned source = 0; source < 2; ++source) {
      for (const bool b : {false, true}) {
        for (const auto status : flags) {
          for (const auto scale : scales) {
            result.push_back({.value = value,
                              .field = source,
                              .status = status | scale,
                              .destination_b = b,
                              .count = true});
          }
        }
      }
    }
  }
  return result;
}

void append_shifts(std::vector<Input> &result) {
  constexpr std::array<std::uint64_t, 16> values{0,
                                                 1,
                                                 0x7fffff,
                                                 0x800000,
                                                 0xffffff,
                                                 0x1000000,
                                                 0x400000000000,
                                                 0x7fffffffffff,
                                                 0x800000000000,
                                                 0xff800000000000,
                                                 0xfffffffffffffe,
                                                 accumulator_mask,
                                                 sign_bit,
                                                 sign_bit + 1,
                                                 sign_bit - 1,
                                                 other_accumulator};
  constexpr std::array shifts{-55, -48, -47, -31, -1, 0, 1, 7, 8, 31, 47, 55, 56};
  for (const auto value : values) {
    for (const auto shift : shifts) {
      for (unsigned source = 2; source < 8; ++source) {
        for (const bool b : {false, true}) {
          for (const auto status : flags) {
            for (const auto scale : scales) {
              result.push_back({.value = value,
                                .shift = shift,
                                .field = source,
                                .status = status | scale,
                                .destination_b = b});
            }
          }
        }
      }
    }
  }
}

void run_matrix(const std::span<const Input> inputs) {
  for (std::size_t begin = 0; begin < inputs.size(); begin += batch_size) {
    const auto batch = inputs.subspan(begin, std::min(batch_size, inputs.size() - begin));
    DspMachine machine;
    DspProgram code;
    for (const auto &input : batch) {
      seed(code, input);
      code.word(opcode(input));
    }
    code.install(machine.bus());
    for (const auto &input : batch) {
      for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
      verify(machine, input);
    }
  }
}

void normalization_chains() {
  for (const auto value : count_values()) {
    const Input count{
        .value = value, .status = carry | limit, .destination_b = true, .count = true};
    DspMachine machine;
    DspProgram code;
    seed(code, count);
    code.word(opcode(count));
    const Input shift{.value = value,
                      .shift = leading_count(value),
                      .field = 3,
                      .status = expected(count).status};
    code.word(opcode(shift));
    code.install(machine.bus());
    for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
    verify(machine, count);
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 3; ++replay) {
      if (replay != 0) { machine.restore(saved); }
      verify(machine, shift, false);
    }
  }
}

void require_quarantine(DspMachine &machine) {
  try {
    static_cast<void>(machine.checkpoint());
  } catch (const Fault &) { return; }
  throw Fault("A rejected native instruction must not become a trusted checkpoint");
}

void reject(const Input &input, const std::string &reason) {
  DspMachine machine;
  DspProgram code;
  seed(code, input);
  code.word(opcode(input));
  code.install(machine.bus());
  for (unsigned step = 0; step < setup_steps; ++step) { machine.cpu().step(); }
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().debug_state();
  try {
    machine.cpu().step();
  } catch (const Fault &fault) {
    const std::string message(fault.what());
    require(message.contains(reason) && message.contains("pc=") && message.contains("omr=") &&
                machine.cpu().state().a == before.cpu.a && machine.cpu().state().b == before.cpu.b,
            "Unqualified fast normalization rejects before accumulator effects");
    require_quarantine(machine);
    machine.restore(saved);
    require(machine.cpu().debug_state() == before, "Trusted recovery retains exact native state");
    return;
  }
  throw Fault("Unqualified fast normalization silently succeeded");
}

void rejection_checks() {
  for (unsigned field = 2; field < 8; ++field) {
    for (const bool b : {false, true}) {
      for (const int shift : {-56, 57, -0x800000, 0x7fffff}) {
        reject({.value = other_accumulator, .shift = shift, .field = field, .destination_b = b},
               "NORMF shift count=");
      }
      reject(
          {.value = other_accumulator, .field = field, .status = arithmetic_16, .destination_b = b},
          "NORMF sixteen-bit arithmetic");
    }
  }
  for (unsigned field = 0; field < 2; ++field) {
    for (const bool b : {false, true}) {
      reject({.value = other_accumulator,
              .field = field,
              .status = arithmetic_16,
              .destination_b = b,
              .count = true},
             "CLB sixteen-bit arithmetic");
    }
  }
}

void preserved_full_width_input() {
  // Own literal regression: X0=56, A=$80:000000:000000, C=1; NORMF X0,A.
  constexpr std::array<std::uint8_t, 45> bytes{
      0, 0, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0,    0,    0, 0, 0, 0,    0x80, 0,   0,
      0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0, 0, 1, 0x0c, 0x1e, 0x28};
  const auto input = decode_instruction_input(bytes);
  if (!input) { throw Fault("The full-width NORMF regression must decode"); }
  const auto result = execute_instruction_input(input.value(), 1);
  require(result.stop == InstructionStop::budget && result.steps == 1 &&
              result.state.cpu.a == accumulator_mask &&
              result.state.cpu.sr == (masked_status | carry | negative | unnormalized),
          "Literal +56 regression sign-fills all 56 bits and preserves carry");
}
}

int main() {
  try {
    preserved_full_width_input();
    auto inputs = count_matrix();
    append_shifts(inputs);
    run_matrix(inputs);
    normalization_chains();
    rejection_checks();
    std::cout << inputs.size() << " independent CLB/NORMF cases, 114 native chains, "
              << "228 suffix replays and 64 strict recoveries passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
