// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"

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
void test_no_shift_refreshes_flags() {
  DspBus bus;
  DspProgram program;
  program.immediate("r1", 0);
  program.emit("clr a");
  constexpr unsigned initial_value = 0x123;
  program.immediate("a1", initial_value);
  const auto first_norm = program.pc();
  for (unsigned index = 0; index < 3; ++index) { program.emit("norm r1,a"); }
  program.install(bus);
  DspCore cpu(bus);
  while (cpu.state().pc != first_norm) { cpu.step(); }
  require((cpu.state().sr & zero) != 0 &&
              cpu.state().a == (std::uint64_t{initial_value} << word_bits),
          "Own MOVEs change A without replacing the prior CLR zero flag");
  cpu.step();
  require(cpu.state().a == (std::uint64_t{initial_value} << word_bits) && cpu.state().r[1] == 0 &&
              cpu.state().sr == (masked_status | unnormalized),
          "NORM's no-shift path must refresh result flags, allowing the next iteration");
  cpu.step();
  constexpr unsigned second_value = 0x246;
  require(cpu.state().a == (std::uint64_t{second_value} << word_bits) &&
              cpu.state().r[1] == word_mask,
          "Second NORM consumes the refreshed flags and decrements R1");
  cpu.step();
  constexpr unsigned final_value = 0x48c;
  require(cpu.state().a == (std::uint64_t{final_value} << word_bits) &&
              cpu.state().r[1] == word_mask - 1 && cpu.state().sr == (masked_status | unnormalized),
          "Three native NORM instructions match the independent hardware observation");
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

unsigned initialize(DspProgram &program, const NormInput input) {
  for (const bool accumulator_b : {false, true}) {
    const auto value = accumulator_b == input.accumulator_b ? input.value : other_accumulator;
    const std::string name = accumulator_b ? "b" : "a";
    program.immediate(name + '0', value & word_mask);
    program.immediate(name + '1', (value >> word_bits) & word_mask);
    program.immediate(name + '2', value >> data_bits);
  }
  for (unsigned reg = 0; reg < 8; ++reg) {
    const auto name = std::to_string(reg);
    program.immediate("r" + name,
                      reg == input.address_register ? input.counter : address_seed + reg);
    program.immediate("n" + name, offset_seed + reg);
    program.immediate("m" + name, modifier_seed);
  }
  program.immediate("sr", input.status);
  const auto instruction = program.pc();
  program.emit("norm r" + std::to_string(input.address_register) +
               (input.accumulator_b ? ",b" : ",a"));
  return instruction;
}

void check_result(const DspState &after, const NormInput input, const unsigned pc) {
  const auto expected = expected_norm(input);
  const auto actual = input.accumulator_b ? after.b : after.a;
  const auto other = input.accumulator_b ? after.a : after.b;
  const auto context = "NORM at " + number(pc) + " value=" + number(input.value) +
                       " input-sr=" + number(input.status);
  require(after.pc == pc + 1 && after.sr == expected.status && actual == expected.value,
          context + " has unexpected PC/accumulator/flags: sr=" + number(after.sr) +
              " expected=" + number(expected.status));
  require(other == other_accumulator && after.sp == 0 && after.lc == 0 && after.omr == 5,
          context + " modified another accumulator or control state");
  for (unsigned reg = 0; reg < 8; ++reg) {
    require(after.r[reg] ==
                    (reg == input.address_register ? expected.counter : address_seed + reg) &&
                after.n[reg] == offset_seed + reg && after.m[reg] == modifier_seed,
            context + " modified address state incorrectly");
  }
}

void run_batch(DspBus &bus, DspCore &cpu, const std::span<const NormInput> batch) {
  DspProgram program;
  std::vector<unsigned> instructions;
  for (const auto input : batch) { instructions.push_back(initialize(program, input)); }
  program.install(bus);
  cpu.reset_spi_boot();
  for (std::size_t index = 0; index < batch.size(); ++index) {
    unsigned setup = 0;
    while (cpu.state().pc != instructions[index]) {
      constexpr unsigned setup_limit = 40;
      require(++setup <= setup_limit,
              "NORM diagnostic setup exceeded its literal instruction bound");
      cpu.step();
    }
    cpu.step();
    check_result(cpu.state(), batch[index], instructions[index]);
  }
}

void test_norm_matrix() {
  const auto cases = inputs();
  constexpr unsigned expected_cases = 4608;
  require(cases.size() == expected_cases, "NORM matrix covers every declared vector combination");
  DspBus bus;
  DspCore cpu(bus);
  constexpr unsigned batch_size = 32;
  for (std::size_t offset = 0; offset < cases.size(); offset += batch_size) {
    run_batch(bus, cpu, std::span(cases).subspan(offset, batch_size));
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
