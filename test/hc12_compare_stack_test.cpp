// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hc12_cycle_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using blofeld::test::cycle_entry;
using blofeld::test::cycle_pointer;
using blofeld::test::cycle_source;
using blofeld::test::CycleFixture;
using blofeld::test::require_cycle;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned initial_d = 0x3456;
constexpr unsigned initial_y = 0x4567;
constexpr unsigned direct_address = 0x40;
constexpr unsigned nop = 0xa7;

struct CompareVector {
  std::uint16_t stack;
  std::uint16_t operand;
  std::uint8_t flags;
};

// Independent literal 16-bit subtraction boundaries, S12CPUV2 CPS p. 158.
constexpr std::array<CompareVector, 9> vectors{{
    {.stack = 0x1234, .operand = 0x1234, .flags = 4},
    {.stack = 0x1233, .operand = 0x1234, .flags = 9},
    {.stack = 0x1234, .operand = 0xffff, .flags = 1},
    {.stack = 0x8000, .operand = 1, .flags = 2},
    {.stack = 0x7fff, .operand = 0xffff, .flags = 11},
    {.stack = 0xffff, .operand = 0xffff, .flags = 4},
    {.stack = 0, .operand = 1, .flags = 9},
    {.stack = 0x7fff, .operand = 0x8000, .flags = 11},
    {.stack = 0x8000, .operand = 0x7fff, .flags = 2},
}};

struct CompareMode {
  std::array<std::uint8_t, 4> code;
  unsigned length;
  unsigned cycles;
  unsigned read_cycle;
  unsigned address;
  std::uint16_t x;
  std::uint16_t d;
  bool indirect;
};

constexpr std::array<CompareMode, 8> modes{{
    {.code = {0x8f, 0, 0, 0},
     .length = 3,
     .cycles = 2,
     .read_cycle = 0,
     .address = cycle_source,
     .x = cycle_source,
     .d = initial_d,
     .indirect = false},
    {.code = {0xbf, 0x12, 0x01, 0},
     .length = 3,
     .cycles = 3,
     .read_cycle = 1,
     .address = cycle_source,
     .x = cycle_source,
     .d = initial_d,
     .indirect = false},
    {.code = {0x9f, 0x40, 0, 0},
     .length = 2,
     .cycles = 3,
     .read_cycle = 1,
     .address = direct_address,
     .x = cycle_source,
     .d = initial_d,
     .indirect = false},
    {.code = {0xaf, 0, 0, 0},
     .length = 2,
     .cycles = 3,
     .read_cycle = 1,
     .address = cycle_source,
     .x = cycle_source,
     .d = initial_d,
     .indirect = false},
    {.code = {0xaf, 0xe0, 0x11, 0},
     .length = 3,
     .cycles = 3,
     .read_cycle = 1,
     .address = cycle_source,
     .x = 0x11f0,
     .d = initial_d,
     .indirect = false},
    {.code = {0xaf, 0xe2, 0x01, 0x11},
     .length = 4,
     .cycles = 4,
     .read_cycle = 2,
     .address = cycle_source,
     .x = 0x10f0,
     .d = initial_d,
     .indirect = false},
    {.code = {0xaf, 0xe7, 0, 0},
     .length = 2,
     .cycles = 6,
     .read_cycle = 4,
     .address = cycle_source,
     .x = 0x1401,
     .d = 1,
     .indirect = true},
    {.code = {0xaf, 0xe3, 0, 0x10},
     .length = 4,
     .cycles = 6,
     .read_cycle = 4,
     .address = cycle_source,
     .x = 0x13f2,
     .d = initial_d,
     .indirect = true},
}};

struct CompareCase {
  CompareMode mode;
  CompareVector vector;
  std::uint8_t before_flags;
  unsigned alignment;
};

void check_compare(CycleFixture &fixture, const CompareCase &item) {
  auto code = item.mode.code;
  constexpr unsigned immediate = 0x8f;
  if (code[0] == immediate) {
    code[1] = static_cast<std::uint8_t>(item.vector.operand >> 8U);
    code[2] = static_cast<std::uint8_t>(item.vector.operand & byte_mask);
  }
  fixture.memory.bytes[item.mode.address] = static_cast<std::uint8_t>(item.vector.operand >> 8U);
  fixture.memory.bytes[item.mode.address + 1] =
      static_cast<std::uint8_t>(item.vector.operand & byte_mask);
  fixture.memory.bytes[cycle_pointer] = cycle_source >> 8U;
  fixture.memory.bytes[cycle_pointer + 1] = cycle_source & byte_mask;
  const auto address = static_cast<std::uint16_t>(cycle_entry + item.alignment);
  fixture.install(std::span(code).first(item.mode.length),
                  {.d = item.mode.d,
                   .x = item.mode.x,
                   .y = initial_y,
                   .sp = item.vector.stack,
                   .ccr = item.before_flags},
                  address);
  const auto begin = fixture.clocks.size();
  fixture.cpu.step();
  const auto state = fixture.cpu.state();
  constexpr unsigned upper_flags = 0xf0;
  require_cycle(state.pc == address + item.mode.length && state.sp == item.vector.stack &&
                    state.d == item.mode.d && state.x == item.mode.x && state.y == initial_y &&
                    state.ccr == ((item.before_flags & upper_flags) | item.vector.flags) &&
                    fixture.clocks.size() == begin + item.mode.cycles,
                "CPS must retain SP/other registers and match literal flags/length/manual cycles");
  unsigned operand_cycles = 0;
  for (auto index = begin; index < fixture.clocks.size(); ++index) {
    if (fixture.clocks.at(index).address) { ++operand_cycles; }
  }
  const unsigned expected_operands = item.mode.read_cycle == 0 ? 0U : 1U;
  require_cycle(operand_cycles == expected_operands + (item.mode.indirect ? 1U : 0U),
                "Immediate CPS uses instruction bytes directly; memory CPS reads only its data");
  if (item.mode.read_cycle != 0) {
    fixture.check_operand(begin + item.mode.read_cycle - 1,
                          {.kind = 'R', .address = item.mode.address});
  }
  if (item.mode.indirect) {
    fixture.check_operand(begin + 1, {.kind = 'I', .address = cycle_pointer});
  }
  require_cycle(fixture.memory.read_memory(
                    {.address = static_cast<std::uint16_t>(item.mode.address), .size = 2}) ==
                    item.vector.operand,
                "CPS never stores to its compared memory operand");
}

void test_compare_modes_and_flags() {
  CycleFixture fixture;
  constexpr std::array<unsigned, 2> preserved_flags{0xc0, 0xf0};
  for (const auto &mode : modes) {
    for (const auto &vector : vectors) {
      for (const auto upper : preserved_flags) {
        for (unsigned low = 0; low < 16; ++low) {
          for (unsigned alignment = 0; alignment < 2; ++alignment) {
            check_compare(fixture,
                          {.mode = mode,
                           .vector = vector,
                           .before_flags = static_cast<std::uint8_t>(upper | low),
                           .alignment = alignment});
          }
        }
      }
    }
  }
}

void test_partial_instruction_fetch() {
  constexpr std::array<std::uint8_t, 2> opcodes{0x8f, 0xbf};
  constexpr unsigned ram_code = 0x1200;
  constexpr unsigned reset_vector_offset = 0x7ffe;
  constexpr unsigned stack = 0x2000;
  constexpr unsigned masked = 0xd0;
  for (const auto opcode : opcodes) {
    for (unsigned missing = 1; missing < 3; ++missing) {
      std::array<std::uint8_t, 32768> flash{};
      flash[reset_vector_offset] = cycle_entry >> 8U;
      blofeld::McuBus memory(flash);
      unsigned clocks = 0;
      blofeld::Hc12CycleBus timed(memory, [&clocks](auto /*cycle*/) { ++clocks; });
      blofeld::Hc12 cpu(timed, BLOFELD_DEFAULT_SLA);
      cpu.reset();
      memory.write(ram_code, opcode);
      memory.write(static_cast<std::uint16_t>(ram_code + 3 - missing), 0);
      cpu.set_state({.pc = ram_code, .sp = stack, .ccr = masked});
      const auto before_clocks = clocks;
      const auto revision = memory.revision();
      std::string error;
      try {
        cpu.step();
      } catch (const blofeld::Fault &fault) { error = fault.what(); }
      require_cycle(
          error.contains("instruction fetch at " + blofeld::hex(ram_code + missing)) &&
              error.contains("pc=0x1200") && error.contains("ccr=0xd0") &&
              cpu.state().pc == ram_code && cpu.state().sp == stack && cpu.state().ccr == masked &&
              clocks == before_clocks && memory.revision() == revision,
          "Every CPS operand byte requires execute permission before any clock/state effect");
    }
  }
}

void test_conditional_skip_overlap() {
  CycleFixture fixture;
  // Own compiler-style idiom: a taken BEQ enters the CPS immediate operand as
  // LDAA #22; the other path executes CPS #8622 while retaining A=11.
  constexpr std::array<std::uint8_t, 10> code{0xc1, 2, 0x27, 3, 0x86, 0x11, 0x8f, 0x86, 0x22, nop};
  fixture.memory.install(code, cycle_entry);
  constexpr std::array<unsigned, 3> inputs{1, 2, 1};
  constexpr unsigned stack = 0x2000;
  constexpr unsigned flags = 0xc0;
  constexpr unsigned other_flags = 0xcb;
  constexpr unsigned result_a = 0x11;
  constexpr unsigned alternate_a = 0x22;
  for (const auto input : inputs) {
    fixture.cpu.set_state({.pc = cycle_entry,
                           .d = static_cast<std::uint16_t>(input),
                           .x = cycle_source,
                           .y = initial_y,
                           .sp = stack,
                           .ccr = flags});
    const auto begin = fixture.clocks.size();
    constexpr unsigned maximum_steps = 8;
    for (unsigned step = 0;
         step < maximum_steps && fixture.cpu.state().pc != cycle_entry + code.size();
         ++step) {
      fixture.cpu.step();
    }
    const auto state = fixture.cpu.state();
    const unsigned expected_a = input == 2 ? alternate_a : result_a;
    require_cycle(
        state.pc == cycle_entry + code.size() && state.d == (expected_a * 256U) + input &&
            state.sp == stack && state.x == cycle_source && state.y == initial_y &&
            state.ccr == (input == 2 ? flags : other_flags) && fixture.clocks.size() == begin + 6,
        "Overlapping CPS/alternative-load entries preserve native control flow and state");
  }
}
}

int main() {
  try {
    test_compare_modes_and_flags();
    test_conditional_skip_overlap();
    test_partial_instruction_fetch();
    std::cout << "CPS literal flag/address/cycle vectors and overlapping branch entries passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
