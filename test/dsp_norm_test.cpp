// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_norm_expectations.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
namespace native = dma_test;
using Program = native::NativeProgram;
// FM13-118/131/146: long immediate data/control MOVE and NORM Rn,D.
constexpr unsigned move_r0 = 0x60f400, move_n0 = 0x70f400, move_m0 = 0x05f420;
constexpr unsigned move_sr = 0x05f439, move_a1 = 0x54f400;
constexpr unsigned clear_sr = 0x0500b9; // FM13-131: eight-bit MOVEC #0,SR clears SC.
constexpr unsigned norm_base = 0x01d815, clear_a = 0x200013;
constexpr unsigned register_shift = 16, norm_register_shift = 8;
constexpr unsigned norm_cycle_units = 5;           // FM tableA-1, A-7.
constexpr unsigned mode_settling_instructions = 3; // FM4.2/5.4.1: SC transition restriction.
constexpr std::array accumulator_seeds{
    0x50f400U, 0x54f400U, 0x52f400U, 0x51f400U, 0x55f400U, 0x53f400U};

void immediate(Program &program, const unsigned opcode, const unsigned value) {
  program.word(opcode);
  program.word(value);
}

unsigned norm(const NormInput input) {
  constexpr unsigned accumulator_b = 8;
  return norm_base | (input.address_register << norm_register_shift) |
         (input.accumulator_b ? accumulator_b : 0U);
}

void test_no_shift_refreshes_flags() {
  OwnDspMachine machine(native::native_clocks);
  Program program;
  immediate(program, move_r0 | (1U << register_shift), 0);
  program.word(clear_a);
  constexpr unsigned initial_value = 0x123;
  immediate(program, move_a1, initial_value);
  const auto first_norm = program.pc();
  for (unsigned index = 0; index < 3; ++index) { program.word(norm({.address_register = 1})); }
  native::reset_native(machine, program);
  while (machine.cpu().registers().pc != first_norm) { native::step_native(machine); }
  require((machine.cpu().registers().sr & zero) != 0 &&
              machine.cpu().registers().a == (std::uint64_t{initial_value} << word_bits),
          "Own MOVEs change A without replacing the prior CLR zero flag");
  native::step_native(machine);
  require(machine.cpu().registers().a == (std::uint64_t{initial_value} << word_bits) &&
              machine.cpu().registers().r[1] == 0 &&
              machine.cpu().registers().sr == (masked_status | unnormalized),
          "NORM's no-shift path must refresh result flags, allowing the next iteration");
  native::step_native(machine);
  constexpr unsigned second_value = 0x246;
  require(machine.cpu().registers().a == (std::uint64_t{second_value} << word_bits) &&
              machine.cpu().registers().r[1] == word_mask,
          "Second NORM consumes the refreshed flags and decrements R1");
  native::step_native(machine);
  constexpr unsigned final_value = 0x48c;
  require(machine.cpu().registers().a == (std::uint64_t{final_value} << word_bits) &&
              machine.cpu().registers().r[1] == word_mask - 1 &&
              machine.cpu().registers().sr == (masked_status | unnormalized),
          "Three native NORM instructions match the independent iteration expectations");
}

void append_flag_modes(std::vector<NormInput> &result, const NormInput base) {
  for (const unsigned preserved : {0U, carry, sticky | limit, sticky | limit | carry}) {
    auto input = base;
    input.status |= preserved;
    input.status |= value_flags(input);
    result.push_back(input);
  }
}

void append_address_modes(std::vector<NormInput> &result, const NormInput base) {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (const bool compatible : {false, true}) {
      auto input = base;
      input.address_register = reg;
      input.status |= compatible ? compatibility : 0;
      append_flag_modes(result, input);
    }
  }
}

std::vector<NormInput> inputs() {
  constexpr std::array<std::uint64_t, 12> values{0,
                                                 1,
                                                 0xffffff,
                                                 0x000123000000,
                                                 0x400000000000,
                                                 0x7fffffffffff,
                                                 0x800000000000,
                                                 0x1000000000000,
                                                 0x7fffffffffffff,
                                                 0x80000000000000,
                                                 0xff800000000000,
                                                 0xffffffffffffff};
  std::vector<NormInput> result;
  for (const auto value : values) {
    const auto counter = (value & sign_bit) != 0 ? word_mask : 0;
    for (const unsigned scaling : {0U, scale_down, scale_up}) {
      for (const bool accumulator_b : {false, true}) {
        append_address_modes(result,
                             {.value = value,
                              .counter = counter,
                              .status = masked_status | scaling | overflow,
                              .accumulator_b = accumulator_b});
      }
    }
  }
  return result;
}

unsigned initialize(Program &program, const NormInput input) {
  program.word(clear_sr);
  for (unsigned index = 0; index < mode_settling_instructions; ++index) { program.word(0); }
  for (const bool accumulator_b : {false, true}) {
    const auto value = accumulator_b == input.accumulator_b ? input.value : other_accumulator;
    const unsigned first = accumulator_b ? 3U : 0U;
    immediate(program, accumulator_seeds[first], static_cast<unsigned>(value & word_mask));
    immediate(program,
              accumulator_seeds[first + 1],
              static_cast<unsigned>((value >> word_bits) & word_mask));
    immediate(program, accumulator_seeds[first + 2], static_cast<unsigned>(value >> data_bits));
  }
  for (unsigned reg = 0; reg < 8; ++reg) {
    immediate(program,
              move_r0 | (reg << register_shift),
              reg == input.address_register ? input.counter : address_seed + reg);
    immediate(program, move_n0 | (reg << register_shift), offset_seed + reg);
    immediate(program, move_m0 | reg, modifier_seed);
  }
  immediate(program, move_sr, input.status);
  for (unsigned index = 0; index < mode_settling_instructions; ++index) { program.word(0); }
  const auto instruction = program.pc();
  program.word(norm(input));
  return instruction;
}

void check_result(const dsp56300::Registers &after, const NormInput input, const unsigned pc) {
  const auto expected = expected_norm(input);
  const auto actual = input.accumulator_b ? after.b : after.a;
  const auto other = input.accumulator_b ? after.a : after.b;
  const auto context = "NORM at " + number(pc) + " value=" + number(input.value) +
                       " input-sr=" + number(input.status);
  require(after.pc == pc + 1 && after.sr == expected.status && actual == expected.value,
          context + " has unexpected PC/accumulator/flags: sr=" + number(after.sr) +
              " expected=" + number(expected.status));
  require(other == other_accumulator && after.sp == 0 && after.lc == 0 && after.omr == reset_omr,
          context + " modified another accumulator or control state");
  for (unsigned reg = 0; reg < 8; ++reg) {
    require(after.r[reg] ==
                    (reg == input.address_register ? expected.counter : address_seed + reg) &&
                after.n[reg] == offset_seed + reg && after.m[reg] == modifier_seed,
            context + " modified address state incorrectly");
  }
}

void run_batch(OwnDspMachine &machine, const std::span<const NormInput> batch) {
  Program program;
  std::vector<unsigned> instructions;
  for (const auto input : batch) { instructions.push_back(initialize(program, input)); }
  native::reset_native(machine, program);
  for (std::size_t index = 0; index < batch.size(); ++index) {
    unsigned setup = 0;
    while (machine.cpu().registers().pc != instructions[index]) {
      constexpr unsigned setup_limit = 40;
      require(++setup <= setup_limit,
              "NORM diagnostic setup exceeded its literal instruction bound");
      native::step_native(machine);
    }
    const auto before = machine.cpu().progress();
    native::step_native(machine);
    check_result(machine.cpu().registers(), batch[index], instructions[index]);
    require(machine.cpu().progress().instructions == before.instructions + 1 &&
                machine.cpu().progress().estimated_cycles ==
                    before.estimated_cycles + norm_cycle_units,
            "NORM retires once with its independent FM tableA-1 base cost");
  }
}

void test_norm_matrix() {
  const auto cases = inputs();
  constexpr unsigned expected_cases = 4608;
  require(cases.size() == expected_cases, "NORM matrix covers every declared vector combination");
  OwnDspMachine machine(native::native_clocks);
  constexpr unsigned batch_size = 32;
  for (std::size_t offset = 0; offset < cases.size(); offset += batch_size) {
    run_batch(machine, std::span(cases).subspan(offset, batch_size));
  }
  std::cout << cases.size()
            << " independent NORM value/scaling/A-B/Rn/compatibility/flag vectors passed\n";
}
}
}

int main() {
  try {
    blofeld::test::test_no_shift_refreshes_flags();
    blofeld::test::test_norm_matrix();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
