// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hc12_cycle_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using blofeld::test::cycle_destination;
using blofeld::test::cycle_entry;
using blofeld::test::cycle_external_destination;
using blofeld::test::cycle_flags;
using blofeld::test::cycle_pointer;
using blofeld::test::cycle_source;
using blofeld::test::cycle_stack;
using blofeld::test::CycleFixture;
using blofeld::test::require_cycle;
constexpr unsigned initial_d = 0x3456;
constexpr unsigned move_word = 0xa53c;
constexpr unsigned move_byte = 0xa5;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned return_opcode = 0x3d;

struct MoveCase {
  std::array<std::uint8_t, 6> code;
  unsigned length;
  unsigned cycles;
  unsigned read_cycle;
  unsigned write_cycle;
  bool indexed_destination;
  bool word;
};

constexpr std::array<MoveCase, 12> move_cases{{
    {.code = {0x18, 0x0b, 0xa5, 0x13, 0x41, 0},
     .length = 5,
     .cycles = 4,
     .read_cycle = 0,
     .write_cycle = 3,
     .indexed_destination = false,
     .word = false},
    {.code = {0x18, 0x08, 0x40, 0xa5, 0, 0},
     .length = 4,
     .cycles = 4,
     .read_cycle = 0,
     .write_cycle = 3,
     .indexed_destination = true,
     .word = false},
    {.code = {0x18, 0x0c, 0x12, 0x01, 0x13, 0x41},
     .length = 6,
     .cycles = 6,
     .read_cycle = 2,
     .write_cycle = 4,
     .indexed_destination = false,
     .word = false},
    {.code = {0x18, 0x09, 0x40, 0x12, 0x01, 0},
     .length = 5,
     .cycles = 5,
     .read_cycle = 3,
     .write_cycle = 5,
     .indexed_destination = true,
     .word = false},
    {.code = {0x18, 0x0d, 0x00, 0x13, 0x41, 0},
     .length = 5,
     .cycles = 5,
     .read_cycle = 2,
     .write_cycle = 4,
     .indexed_destination = false,
     .word = false},
    {.code = {0x18, 0x0a, 0x00, 0x40, 0, 0},
     .length = 4,
     .cycles = 5,
     .read_cycle = 2,
     .write_cycle = 4,
     .indexed_destination = true,
     .word = false},
    {.code = {0x18, 0x03, 0xa5, 0x3c, 0x13, 0x41},
     .length = 6,
     .cycles = 5,
     .read_cycle = 0,
     .write_cycle = 3,
     .indexed_destination = false,
     .word = true},
    {.code = {0x18, 0x00, 0x40, 0xa5, 0x3c, 0},
     .length = 5,
     .cycles = 4,
     .read_cycle = 0,
     .write_cycle = 4,
     .indexed_destination = true,
     .word = true},
    {.code = {0x18, 0x04, 0x12, 0x01, 0x13, 0x41},
     .length = 6,
     .cycles = 6,
     .read_cycle = 2,
     .write_cycle = 4,
     .indexed_destination = false,
     .word = true},
    {.code = {0x18, 0x01, 0x40, 0x12, 0x01, 0},
     .length = 5,
     .cycles = 5,
     .read_cycle = 3,
     .write_cycle = 5,
     .indexed_destination = true,
     .word = true},
    {.code = {0x18, 0x05, 0x00, 0x13, 0x41, 0},
     .length = 5,
     .cycles = 5,
     .read_cycle = 2,
     .write_cycle = 4,
     .indexed_destination = false,
     .word = true},
    {.code = {0x18, 0x02, 0x00, 0x40, 0, 0},
     .length = 4,
     .cycles = 5,
     .read_cycle = 2,
     .write_cycle = 4,
     .indexed_destination = true,
     .word = true},
}};

void test_move_modes() {
  // S12CPUV2 MOVB/MOVW: literal object encodings, lengths and operand slots.
  CycleFixture fixture;
  constexpr unsigned preserved_flags = 0xf5;
  for (const auto &item : move_cases) {
    fixture.memory.bytes[cycle_source] = move_byte;
    fixture.memory.bytes[cycle_source + 1] = move_word & byte_mask;
    fixture.memory.bytes[cycle_destination] = 0;
    fixture.memory.bytes[cycle_external_destination] = 0;
    fixture.install(std::span(item.code).first(item.length),
                    {.d = initial_d,
                     .x = cycle_source,
                     .y = cycle_destination,
                     .sp = cycle_stack,
                     .ccr = preserved_flags});
    const auto begin = fixture.clocks.size();
    fixture.cpu.step();
    const auto state = fixture.cpu.state();
    const auto destination =
        item.indexed_destination ? cycle_destination : cycle_external_destination;
    const auto width = item.word ? 2U : 1U;
    require_cycle(
        fixture.clocks.size() == begin + item.cycles && state.pc == cycle_entry + item.length &&
            state.d == initial_d && state.x == cycle_source && state.y == cycle_destination &&
            state.sp == cycle_stack && state.ccr == preserved_flags &&
            fixture.memory.read_memory({.address = static_cast<std::uint16_t>(destination),
                                        .size = width}) == (item.word ? move_word : move_byte),
        "All six native MOVB/MOVW modes retain independent data, CCR and cycle results");
    if (item.read_cycle != 0) {
      fixture.check_operand(begin + item.read_cycle - 1,
                            {.kind = item.word ? 'R' : 'r', .address = cycle_source});
    }
    fixture.check_operand(begin + item.write_cycle - 1,
                          {.kind = item.word ? 'W' : 'w', .address = destination});
  }
}

struct ModifyCase {
  std::uint8_t low_opcode;
  std::uint8_t input;
  std::uint8_t result;
  std::uint8_t ccr;
};

struct AddressMode {
  std::array<std::uint8_t, 3> operand;
  unsigned length;
  unsigned modify_cycles;
  unsigned read_cycle;
  unsigned write_cycle;
  unsigned test_cycles;
  std::uint16_t x;
  std::uint16_t d;
  bool extended;
  bool indirect;
};

constexpr std::array<AddressMode, 6> modes{{
    {.operand = {0x12, 0x01, 0},
     .length = 2,
     .modify_cycles = 4,
     .read_cycle = 1,
     .write_cycle = 3,
     .test_cycles = 3,
     .x = cycle_source,
     .d = 0,
     .extended = true,
     .indirect = false},
    {.operand = {0x00, 0, 0},
     .length = 1,
     .modify_cycles = 3,
     .read_cycle = 1,
     .write_cycle = 3,
     .test_cycles = 3,
     .x = cycle_source,
     .d = 0,
     .extended = false,
     .indirect = false},
    {.operand = {0xe0, 0x11, 0},
     .length = 2,
     .modify_cycles = 4,
     .read_cycle = 1,
     .write_cycle = 3,
     .test_cycles = 3,
     .x = 0x11f0,
     .d = 0,
     .extended = false,
     .indirect = false},
    {.operand = {0xe2, 0x01, 0x11},
     .length = 3,
     .modify_cycles = 5,
     .read_cycle = 2,
     .write_cycle = 4,
     .test_cycles = 4,
     .x = 0x10f0,
     .d = 0,
     .extended = false,
     .indirect = false},
    {.operand = {0xe7, 0, 0},
     .length = 1,
     .modify_cycles = 6,
     .read_cycle = 4,
     .write_cycle = 6,
     .test_cycles = 6,
     .x = 0x1401,
     .d = 1,
     .extended = false,
     .indirect = true},
    {.operand = {0xe3, 0, 0x10},
     .length = 3,
     .modify_cycles = 6,
     .read_cycle = 4,
     .write_cycle = 6,
     .test_cycles = 6,
     .x = 0x13f2,
     .d = 0,
     .extended = false,
     .indirect = true},
}};

void install_mode(CycleFixture &fixture, const AddressMode &mode, const std::uint8_t opcode) {
  std::array<std::uint8_t, 4> code{opcode, mode.operand[0], mode.operand[1], mode.operand[2]};
  fixture.memory.bytes[cycle_pointer] = cycle_source >> 8U;
  fixture.memory.bytes[cycle_pointer + 1] = cycle_source & byte_mask;
  fixture.install(std::span(code).first(mode.length + 1),
                  {.d = mode.d,
                   .x = mode.x,
                   .y = cycle_destination,
                   .sp = cycle_stack,
                   .ccr = cycle_flags | 1U});
}

void test_modify_and_test_modes() {
  constexpr std::array<ModifyCase, 9> operations{{
      {.low_opcode = 0, .input = 1, .result = 0xff, .ccr = 0xc9},
      {.low_opcode = 1, .input = 0x55, .result = 0xaa, .ccr = 0xc9},
      {.low_opcode = 2, .input = 0x7f, .result = 0x80, .ccr = 0xcb},
      {.low_opcode = 3, .input = 0x80, .result = 0x7f, .ccr = 0xc3},
      {.low_opcode = 4, .input = 1, .result = 0, .ccr = 0xc7},
      {.low_opcode = 5, .input = 0x80, .result = 1, .ccr = 0xc3},
      {.low_opcode = 6, .input = 1, .result = 0x80, .ccr = 0xc9},
      {.low_opcode = 7, .input = 0x81, .result = 0xc0, .ccr = 0xc9},
      {.low_opcode = 8, .input = 0x40, .result = 0x80, .ccr = 0xca},
  }};
  CycleFixture fixture;
  constexpr unsigned indexed_modify = 0x60;
  constexpr unsigned extended_modify = 0x70;
  constexpr unsigned indexed_test = 0xe7;
  constexpr unsigned extended_test = 0xf7;
  for (const auto &mode : modes) {
    for (const auto &operation : operations) {
      fixture.memory.bytes[cycle_source] = operation.input;
      install_mode(fixture,
                   mode,
                   static_cast<std::uint8_t>((mode.extended ? extended_modify : indexed_modify) |
                                             operation.low_opcode));
      const auto begin = fixture.clocks.size();
      fixture.cpu.step();
      const auto state = fixture.cpu.state();
      require_cycle(
          fixture.clocks.size() == begin + mode.modify_cycles &&
              fixture.memory.bytes[cycle_source] == operation.result &&
              state.pc == cycle_entry + mode.length + 1 && state.ccr == operation.ccr &&
              state.x == mode.x && state.d == mode.d && state.sp == cycle_stack,
          "Native memory RMW arithmetic, flags, addressing and literal cycles must match");
      fixture.check_operand(begin + mode.read_cycle - 1, {.kind = 'r', .address = cycle_source});
      fixture.check_operand(begin + mode.write_cycle - 1, {.kind = 'w', .address = cycle_source});
      if (mode.indirect) {
        fixture.check_operand(begin + 1, {.kind = 'I', .address = cycle_pointer});
      }
    }
    constexpr unsigned negative_input = 0x80;
    constexpr unsigned negative_flags = 0xc8;
    fixture.memory.bytes[cycle_source] = negative_input;
    install_mode(
        fixture, mode, static_cast<std::uint8_t>(mode.extended ? extended_test : indexed_test));
    const auto begin = fixture.clocks.size();
    fixture.cpu.step();
    require_cycle(
        fixture.clocks.size() == begin + mode.test_cycles &&
            fixture.cpu.state().ccr == negative_flags &&
            fixture.memory.bytes[cycle_source] == negative_input,
        "Native TST reads without writing, clears V/C and retains its literal mode timing");
    fixture.check_operand(begin + mode.read_cycle - 1, {.kind = 'r', .address = cycle_source});
  }
}

void test_lea_and_relative_call() {
  CycleFixture fixture;
  constexpr std::array<std::uint8_t, 2> stack_address{0x1b, 0x9b}; // LEAS -5,SP.
  fixture.install(stack_address, {.sp = cycle_stack, .ccr = cycle_flags});
  auto begin = fixture.clocks.size();
  fixture.cpu.step();
  require_cycle(fixture.cpu.state().sp == cycle_stack - 5 &&
                    fixture.cpu.state().ccr == cycle_flags && fixture.clocks.size() == begin + 2,
                "LEAS forms a literal address without fetching a data operand");
  constexpr std::array<std::uint8_t, 3> offset9{0x19, 0xe0, 0x61}; // LEAY 97,X.
  fixture.install(offset9, {.x = cycle_source, .sp = cycle_stack, .ccr = cycle_flags});
  begin = fixture.clocks.size();
  fixture.cpu.step();
  constexpr unsigned address9 = 0x1262;
  require_cycle(fixture.cpu.state().y == address9 && fixture.clocks.size() == begin + 2,
                "Nine-bit LEA uses the independent effective address and two cycles");
  constexpr std::array<std::uint8_t, 4> offset16{0x1a, 0xf2, 0x01, 0x23}; // LEAX 291,SP.
  fixture.install(offset16, {.sp = cycle_stack, .ccr = cycle_flags});
  begin = fixture.clocks.size();
  fixture.cpu.step();
  constexpr unsigned address16 = 0x2324;
  require_cycle(fixture.cpu.state().x == address16 && fixture.clocks.size() == begin + 2,
                "Sixteen-bit LEA retains literal SP-relative addressing and two cycles");
  constexpr std::array<std::uint8_t, 3> negative9{0x1b, 0xf1, 0x7f}; // LEAS -129,SP.
  fixture.install(negative9, {.sp = cycle_stack, .ccr = cycle_flags});
  begin = fixture.clocks.size();
  fixture.cpu.step();
  constexpr unsigned negative_address = 0x2180;
  require_cycle(fixture.cpu.state().sp == negative_address && fixture.clocks.size() == begin + 2,
                "Nine-bit LEA offsets must sign-extend independently of the address size");
  constexpr std::array<std::uint8_t, 3> pc9{0x19, 0xf8, 0x0f}; // LEAY 15,PC.
  fixture.install(pc9, {.sp = cycle_stack, .ccr = cycle_flags});
  begin = fixture.clocks.size();
  fixture.cpu.step();
  constexpr unsigned pc_address = 0x8012;
  require_cycle(
      fixture.cpu.state().y == pc_address && fixture.clocks.size() == begin + 2,
      "PC-relative LEA adds its signed offset to the address after the whole instruction");
  constexpr std::array<std::uint8_t, 3> pc_negative9{0x1a, 0xf9, 0xff}; // LEAX -1,PC.
  fixture.install(pc_negative9, {.sp = cycle_stack, .ccr = cycle_flags});
  begin = fixture.clocks.size();
  fixture.cpu.step();
  require_cycle(fixture.cpu.state().x == cycle_entry + 2 && fixture.clocks.size() == begin + 2,
                "Negative PC-relative LEA retains the signed nine-bit displacement");
  constexpr std::array<std::uint8_t, 2> relative_call{7, 0x1e}; // BSR $8020.
  fixture.install(relative_call, {.sp = cycle_stack, .ccr = cycle_flags});
  constexpr unsigned subroutine = 0x8020;
  fixture.memory.bytes[subroutine] = return_opcode;
  begin = fixture.clocks.size();
  fixture.cpu.step();
  require_cycle(fixture.cpu.state().pc == subroutine && fixture.cpu.state().sp == cycle_stack - 2 &&
                    fixture.clocks.size() == begin + 4,
                "BSR stores its actual return PC in the first of four cycles");
  const auto stacked = fixture.clocks.at(begin);
  require_cycle(stacked.kind == 'S' && stacked.address == cycle_stack - 2 && stacked.size == 2,
                "An unaligned internal-RAM stack word remains one native operand cycle");
  fixture.cpu.step();
  require_cycle(fixture.cpu.state().pc == cycle_entry + 2 &&
                    fixture.cpu.state().sp == cycle_stack && fixture.clocks.size() == begin + 9,
                "Native RTS restores the literal return address in five more cycles");
}

struct BranchCase {
  unsigned condition;
  unsigned flags;
  bool taken;
  bool long_branch;
  unsigned alignment;
};

void check_branch(CycleFixture &fixture, const BranchCase &item) {
  constexpr unsigned branch_base = 0x20;
  constexpr unsigned prefix = 0x18;
  constexpr unsigned relative_short = 0xfe;
  constexpr unsigned relative_long_high = 0xff;
  constexpr unsigned relative_long_low = 0xfc;
  const auto opcode = static_cast<std::uint8_t>(branch_base + item.condition);
  const std::array<std::uint8_t, 4> code =
      item.long_branch
          ? std::array<std::uint8_t, 4>{prefix, opcode, relative_long_high, relative_long_low}
          : std::array<std::uint8_t, 4>{opcode, relative_short, 0, 0};
  const unsigned length = item.long_branch ? 4U : 2U;
  const auto address = static_cast<std::uint16_t>(cycle_entry + item.alignment);
  const auto ccr = static_cast<std::uint8_t>(cycle_flags | item.flags);
  fixture.install(
      std::span(code).first(length),
      {.d = initial_d, .x = cycle_source, .y = cycle_destination, .sp = cycle_stack, .ccr = ccr},
      address);
  const auto begin = fixture.clocks.size();
  fixture.cpu.step();
  const auto state = fixture.cpu.state();
  unsigned cycles = item.taken ? 3U : 1U;
  if (item.long_branch) { cycles = item.taken ? 4U : 3U; }
  const bool matches = state.pc == (item.taken ? address : address + length) && state.ccr == ccr &&
                       state.d == initial_d && state.x == cycle_source &&
                       state.y == cycle_destination && state.sp == cycle_stack &&
                       fixture.clocks.size() == begin + cycles;
  if (!matches) {
    throw blofeld::Fault("Branch predicate/vector opcode=" + blofeld::hex(opcode) +
                         " long=" + (item.long_branch ? "1" : "0") +
                         " alignment=" + std::to_string(item.alignment) +
                         " flags=" + blofeld::hex(ccr, 2) + " pc=" + blofeld::hex(state.pc) +
                         " expected-pc=" + blofeld::hex(item.taken ? address : address + length) +
                         " ccr=" + blofeld::hex(state.ccr, 2) +
                         " cycles=" + std::to_string(fixture.clocks.size() - begin) +
                         " expected-cycles=" + std::to_string(cycles));
  }
}

void test_branch_conditions() {
  // Bit n gives the documented result for CCR low nibble n, in opcode order.
  constexpr std::array<unsigned, 16> truth_masks{0xffff,
                                                 0,
                                                 0x0505,
                                                 0xfafa,
                                                 0x5555,
                                                 0xaaaa,
                                                 0x0f0f,
                                                 0xf0f0,
                                                 0x3333,
                                                 0xcccc,
                                                 0x00ff,
                                                 0xff00,
                                                 0xcc33,
                                                 0x33cc,
                                                 0x0c03,
                                                 0xf3fc};
  CycleFixture fixture;
  for (unsigned condition = 0; condition < truth_masks.size(); ++condition) {
    for (unsigned flags = 0; flags < truth_masks.size(); ++flags) {
      const bool taken = (truth_masks.at(condition) & (1U << flags)) != 0;
      for (const bool long_branch : {false, true}) {
        for (unsigned alignment = 0; alignment < 2; ++alignment) {
          check_branch(fixture,
                       {.condition = condition,
                        .flags = flags,
                        .taken = taken,
                        .long_branch = long_branch,
                        .alignment = alignment});
        }
      }
    }
  }
}

struct ArithmeticCase {
  std::array<std::uint8_t, 2> code;
  unsigned length;
  unsigned cycles;
  blofeld::CpuState before;
  blofeld::CpuState after;
  unsigned ccr_mask = byte_mask;
  bool check_d = true;
  bool check_x = true;
};

constexpr std::array<ArithmeticCase, 14> arithmetic_cases{{
    {.code = {0x12, 0},
     .length = 1,
     .cycles = 1,
     .before = {.d = 0x1009, .ccr = 0xd4},
     .after = {.d = 0x0090, .ccr = 0xd5}},
    {.code = {0x13, 0},
     .length = 1,
     .cycles = 3,
     .before = {.d = 0x0101, .y = 0x0102, .ccr = cycle_flags},
     .after = {.d = 0x0302, .y = 1, .ccr = cycle_flags}},
    {.code = {0x11, 0},
     .length = 1,
     .cycles = 11,
     .before = {.d = 7, .x = 3, .ccr = cycle_flags},
     .after = {.d = 1, .x = 3, .y = 2, .ccr = cycle_flags}},
    {.code = {0x18, 0x10},
     .length = 2,
     .cycles = 12,
     .before = {.d = 7, .x = 3, .ccr = cycle_flags},
     .after = {.d = 1, .x = 2, .ccr = cycle_flags}},
    {.code = {0x18, 0x11},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x4000, .x = 0x8000, .ccr = cycle_flags},
     .after = {.x = 0x8000, .ccr = cycle_flags}},
    {.code = {0x18, 0x13},
     .length = 2,
     .cycles = 3,
     .before = {.d = 0xfffe, .y = 3, .ccr = cycle_flags},
     .after = {.d = 0xfffa, .y = 0xffff, .ccr = 0xc9}},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0xfff9, .x = 3, .y = 0xffff, .ccr = cycle_flags},
     .after = {.d = 0xffff, .x = 3, .y = 0xfffe, .ccr = 0xc8}},
    {.code = {0x18, 0x15},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0xfff9, .x = 3, .ccr = cycle_flags},
     .after = {.d = 0xffff, .x = 0xfffe, .ccr = 0xc8}},
    {.code = {0x18, 0x06},
     .length = 2,
     .cycles = 2,
     .before = {.d = 0x7f01, .ccr = cycle_flags},
     .after = {.d = 0x8001, .ccr = 0xea}},
    {.code = {0x18, 0x16},
     .length = 2,
     .cycles = 2,
     .before = {.d = 0x8001, .ccr = cycle_flags},
     .after = {.d = 0x7f01, .ccr = 0xc2}},
    {.code = {0x18, 0x17},
     .length = 2,
     .cycles = 2,
     .before = {.d = 0x8001, .ccr = cycle_flags},
     .after = {.d = 0x8001, .ccr = 0xc2}},
    {.code = {0x18, 0x0e},
     .length = 2,
     .cycles = 2,
     .before = {.d = 0x8001, .ccr = 0xc1},
     .after = {.d = 0x8080, .ccr = 0xc9}},
    {.code = {0x18, 0x0f},
     .length = 2,
     .cycles = 2,
     .before = {.d = 0x0100, .ccr = 0xc1},
     .after = {.ccr = 0xc5}},
    {.code = {0x18, 0x07},
     .length = 2,
     .cycles = 3,
     .before = {.d = 0x0a37, .ccr = cycle_flags},
     .after = {.d = 0x1037, .ccr = cycle_flags},
     .ccr_mask = 0xfd},
}};

void check_arithmetic(CycleFixture &fixture, const ArithmeticCase &item) {
  fixture.install(std::span(item.code).first(item.length), item.before);
  const auto begin = fixture.clocks.size();
  fixture.cpu.step();
  const auto state = fixture.cpu.state();
  const bool matches =
      fixture.clocks.size() == begin + item.cycles && state.pc == cycle_entry + item.length &&
      (!item.check_d || state.d == item.after.d) && (!item.check_x || state.x == item.after.x) &&
      state.y == item.after.y && state.sp == item.before.sp &&
      (state.ccr & item.ccr_mask) == (item.after.ccr & item.ccr_mask);
  if (!matches) {
    throw blofeld::Fault("Arithmetic vector opcode=" + blofeld::hex(item.code[0], 2) +
                         blofeld::hex(item.code[1], 2) + " D=" + blofeld::hex(state.d) +
                         " X=" + blofeld::hex(state.x) + " Y=" + blofeld::hex(state.y) +
                         " CCR=" + blofeld::hex(state.ccr, 2));
  }
}

// Literal signed/unsigned boundaries from S12CPUV2 pp. 170-171, 183, 186-187.
// Undefined flags/remainders are deliberately outside the assertions.
constexpr std::array<ArithmeticCase, 17> division_boundaries{{
    {.code = {0x11, 0},
     .length = 1,
     .cycles = 11,
     .before = {.d = 7, .y = 1, .ccr = 0xfe},
     .after = {.d = 7, .y = 1, .ccr = 0xf1},
     .ccr_mask = 0xf1},
    {.code = {0x11, 0},
     .length = 1,
     .cycles = 11,
     .before = {.d = 7, .x = 1, .y = 1, .ccr = 0xfd},
     .after = {.d = 7, .x = 1, .y = 1, .ccr = 0xf2},
     .ccr_mask = 0xf3},
    {.code = {0x11, 0},
     .length = 1,
     .cycles = 11,
     .before = {.d = 0xffff, .x = 1, .ccr = 0xf7},
     .after = {.x = 1, .y = 0xffff, .ccr = 0xf8}},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0xfff9, .y = 0xffff, .ccr = 0xfe},
     .after = {.d = 0xfff9, .y = 0xffff, .ccr = 0xf1},
     .ccr_mask = 0xf1},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x8000, .x = 1, .ccr = 0xfd},
     .after = {.d = 0x8000, .x = 1, .ccr = 0xf2},
     .ccr_mask = 0xf3},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x7fff, .x = 1, .ccr = 0xff},
     .after = {.x = 1, .y = 0x7fff, .ccr = 0xf0}},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x8000, .x = 1, .y = 0xffff, .ccr = 0xf7},
     .after = {.x = 1, .y = 0x8000, .ccr = 0xf8}},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x7fff, .x = 1, .y = 0xffff, .ccr = 0xfd},
     .after = {.d = 0x7fff, .x = 1, .y = 0xffff, .ccr = 0xf2},
     .ccr_mask = 0xf3},
    {.code = {0x18, 0x14},
     .length = 2,
     .cycles = 12,
     .before = {.x = 0xffff, .y = 0x8000, .ccr = 0xfd},
     .after = {.x = 0xffff, .y = 0x8000, .ccr = 0xf2},
     .ccr_mask = 0xf3},
    {.code = {0x18, 0x15},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0xfff9, .ccr = 0xfe},
     .after = {.d = 0xfff9, .ccr = 0xf1},
     .ccr_mask = 0xf1},
    {.code = {0x18, 0x15},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x8000, .x = 0xffff, .ccr = 0xfd},
     .after = {.ccr = 0xf2},
     .ccr_mask = 0xf3,
     .check_d = false,
     .check_x = false},
    {.code = {0x18, 0x15},
     .length = 2,
     .cycles = 12,
     .before = {.d = 0x8000, .x = 1, .ccr = 0xf7},
     .after = {.x = 0x8000, .ccr = 0xf8}},
    {.code = {0x18, 0x10},
     .length = 2,
     .cycles = 12,
     .before = {.d = 7, .ccr = 0xfe},
     .after = {.x = 0xffff, .ccr = 0xf9},
     .check_d = false},
    {.code = {0x18, 0x10},
     .length = 2,
     .cycles = 12,
     .before = {.d = 2, .x = 3, .ccr = 0xff},
     .after = {.d = 2, .ccr = 0xfc}},
    {.code = {0x18, 0x11},
     .length = 2,
     .cycles = 12,
     .before = {.d = 7, .ccr = 0xfd},
     .after = {.x = 0xffff, .ccr = 0xfb},
     .check_d = false},
    {.code = {0x18, 0x11},
     .length = 2,
     .cycles = 12,
     .before = {.d = 7, .x = 7, .ccr = 0xfd},
     .after = {.x = 0xffff, .ccr = 0xfa},
     .check_d = false},
    {.code = {0x18, 0x11},
     .length = 2,
     .cycles = 12,
     .before = {.x = 1, .ccr = 0xff},
     .after = {.ccr = 0xfc}},
}};

void test_arithmetic_cycles() {
  CycleFixture fixture;
  for (const auto &item : arithmetic_cases) { check_arithmetic(fixture, item); }
  for (const auto &item : division_boundaries) { check_arithmetic(fixture, item); }
}

struct DecimalCase {
  unsigned left;
  unsigned right;
  unsigned carry;
};

void check_decimal(CycleFixture &fixture, const DecimalCase item) {
  constexpr unsigned radix = 10;
  constexpr unsigned range = 100;
  constexpr unsigned preserved = 0xd0;
  constexpr unsigned saved_b = 0x37;
  constexpr unsigned defined_flags = 0xfd;
  const auto a = ((item.left / radix) * 16U) + (item.left % radix);
  const auto b = ((item.right / radix) * 16U) + (item.right % radix);
  const auto before_flags = static_cast<std::uint8_t>(preserved | item.carry);
  const std::array<std::uint8_t, 4> code{0x89, static_cast<std::uint8_t>(b), 0x18, 0x07};
  fixture.install(code,
                  {.d = static_cast<std::uint16_t>((a * 256U) + saved_b),
                   .x = cycle_source,
                   .y = cycle_destination,
                   .sp = cycle_stack,
                   .ccr = before_flags});
  const auto begin = fixture.clocks.size();
  fixture.cpu.step(); // ADCA sets the actual binary result and H/C consumed by DAA.
  fixture.cpu.step();
  const unsigned sum = item.left + item.right + item.carry;
  const unsigned reduced = sum % range;
  const unsigned result = ((reduced / radix) * 16U) + (reduced % radix);
  const unsigned half = ((a & 15U) + (b & 15U) + item.carry) > 15U ? 0x20U : 0U;
  const unsigned negative = (result & 0x80U) != 0 ? 8U : 0U;
  const unsigned zero = result == 0 ? 4U : 0U;
  const unsigned carry = sum >= range ? 1U : 0U;
  const auto state = fixture.cpu.state();
  require_cycle(state.d == result * 256U + saved_b && state.x == cycle_source &&
                    state.y == cycle_destination && state.sp == cycle_stack &&
                    state.pc == cycle_entry + code.size() &&
                    (state.ccr & defined_flags) == (preserved | half | negative | zero | carry) &&
                    fixture.clocks.size() == begin + 4,
                "Actual ADCA/DAA must match independent decimal addition and defined flags");
}

void test_decimal_addition() {
  constexpr unsigned range = 100;
  CycleFixture fixture;
  for (unsigned left = 0; left < range; ++left) {
    for (unsigned right = 0; right < range; ++right) {
      for (unsigned carry = 0; carry < 2; ++carry) {
        check_decimal(fixture, {.left = left, .right = right, .carry = carry});
      }
    }
  }
}

void test_index_counters() {
  constexpr std::array<std::uint8_t, 4> counters{0x08, 0x09, 0x02, 0x03};
  for (const auto opcode : counters) {
    const bool increment = opcode == 0x08 || opcode == 0x02;
    const bool use_x = opcode == 0x08 || opcode == 0x09;
    constexpr std::array<unsigned, 2> before_values{1, 0xffff};
    for (const auto value : before_values) {
      CycleFixture fixture;
      const std::array<std::uint8_t, 1> code{opcode};
      constexpr unsigned initial_flags = 0xff;
      fixture.install(code,
                      {.d = initial_d,
                       .x = static_cast<std::uint16_t>(use_x ? value : cycle_source),
                       .y = static_cast<std::uint16_t>(use_x ? cycle_destination : value),
                       .sp = cycle_stack,
                       .ccr = initial_flags});
      const auto begin = fixture.clocks.size();
      fixture.cpu.step();
      constexpr unsigned word_mask = 0xffff;
      constexpr unsigned other_flags = 0xfb;
      const unsigned expected = (increment ? value + 1 : value - 1) & word_mask;
      const auto state = fixture.cpu.state();
      require_cycle(state.x == (use_x ? expected : cycle_source) &&
                        state.y == (use_x ? cycle_destination : expected) &&
                        state.ccr == (other_flags | (expected == 0 ? 4U : 0U)) &&
                        state.d == initial_d && state.sp == cycle_stack &&
                        state.pc == cycle_entry + 1 && fixture.clocks.size() == begin + 1,
                    "INX/DEX/INY/DEY preserve every flag except Z and take one cycle");
    }
  }
}

void test_ccr_stack() {
  CycleFixture fixture;
  constexpr std::array<std::uint8_t, 4> code{0x39, 0x14, 0x1c, 0x38}; // PSHC; ORCC; PULC.
  constexpr unsigned saved = 0xc1;
  fixture.install(
      code,
      {.d = initial_d, .x = cycle_source, .y = cycle_destination, .sp = cycle_stack, .ccr = saved});
  const auto begin = fixture.clocks.size();
  fixture.cpu.step();
  fixture.check_operand(begin + 1, {.kind = 's', .address = cycle_stack - 1});
  require_cycle(fixture.memory.bytes[cycle_stack - 1] == saved &&
                    fixture.cpu.state().sp == cycle_stack - 1,
                "PSHC saves the actual CCR in the second byte-store cycle");
  fixture.cpu.step();
  constexpr unsigned changed = 0xdd;
  require_cycle(fixture.cpu.state().ccr == changed, "ORCC must modify the live saved flags");
  fixture.cpu.step();
  fixture.check_operand(begin + 3, {.kind = 'u', .address = cycle_stack - 1});
  const auto state = fixture.cpu.state();
  require_cycle(fixture.clocks.size() == begin + 6 && state.ccr == saved &&
                    state.sp == cycle_stack && state.d == initial_d && state.x == cycle_source &&
                    state.y == cycle_destination && state.pc == cycle_entry + code.size(),
                "PULC restores CCR/SP while preserving other state with three more cycles");
}

void test_pc_relative_word_operands() {
  CycleFixture fixture;
  constexpr unsigned negative_address = 0x7f82;
  constexpr unsigned positive_address = 0x8012;
  constexpr unsigned expected = 0xa53c;
  constexpr unsigned low = 0x3c;
  constexpr unsigned high = 0xa5;
  constexpr unsigned carry = 0xc1;
  constexpr unsigned negative_flags = 0xc9;
  fixture.memory.bytes[negative_address] = high;
  fixture.memory.bytes[negative_address + 1] = low;
  constexpr std::array<std::uint8_t, 3> load{0xec, 0xf9, 0x7f}; // LDD -129,PC.
  fixture.install(load, {.x = cycle_source, .sp = cycle_stack, .ccr = carry});
  auto begin = fixture.clocks.size();
  fixture.cpu.step();
  fixture.check_operand(begin, {.kind = 'R', .address = negative_address});
  require_cycle(fixture.cpu.state().d == expected && fixture.cpu.state().ccr == negative_flags &&
                    fixture.cpu.state().x == cycle_source && fixture.clocks.size() == begin + 3,
                "Negative PC-relative word loads add the signed displacement before reading");
  constexpr std::array<std::uint8_t, 3> store{0x6c, 0xf8, 0x0f}; // STD 15,PC.
  fixture.install(store, {.d = expected, .x = cycle_source, .sp = cycle_stack, .ccr = carry});
  begin = fixture.clocks.size();
  fixture.cpu.step();
  fixture.check_operand(begin + 1, {.kind = 'W', .address = positive_address});
  require_cycle(fixture.memory.bytes[positive_address] == high &&
                    fixture.memory.bytes[positive_address + 1] == low &&
                    fixture.cpu.state().x == cycle_source && fixture.clocks.size() == begin + 3,
                "Positive PC-relative word stores add to the complete next instruction address");
}

void test_unqualified_operands_reject() {
  constexpr std::array<std::uint8_t, 4> indexes{0xe0, 0xe2, 0xe3, 0xe7};
  for (const auto post : indexes) {
    CycleFixture fixture;
    constexpr unsigned move_byte_immediate = 0x08;
    const std::array<std::uint8_t, 5> code{0x18, move_byte_immediate, post, 0, 0};
    fixture.install(code, {.d = initial_d, .x = cycle_destination, .ccr = cycle_flags});
    const auto begin = fixture.clocks.size();
    std::string plan_error;
    try {
      static_cast<void>(
          blofeld::hc12_cycle_plan(fixture.memory, {.pc = cycle_entry, .ccr = cycle_flags}));
    } catch (const blofeld::Fault &fault) { plan_error = fault.what(); }
    require_cycle(plan_error.contains("MOV requires a single-byte non-indirect"),
                  "The cycle planner must independently reject unsupported MOV index widths");
    std::string error;
    try {
      fixture.cpu.step();
    } catch (const blofeld::Fault &fault) { error = fault.what(); }
    // The pinned HCS12 decoder already excludes the HCS12X-only forms.
    if (!error.contains("HC12 at 0x8000") || !error.contains("Unable to resolve constructor") ||
        fixture.clocks.size() != begin || fixture.cpu.state().d != initial_d ||
        fixture.cpu.state().x != cycle_destination) {
      throw blofeld::Fault("Unsupported MOV post=" + blofeld::hex(post, 2) + " cycles=" +
                           std::to_string(fixture.clocks.size() - begin) + " error=" + error);
    }
  }
  CycleFixture fixture;
  constexpr std::array<std::uint8_t, 2> daa{0x18, 0x07};
  constexpr unsigned undefined_d = 0xff00;
  constexpr unsigned undefined_flags = 0xe1;
  fixture.install(daa, {.d = undefined_d, .ccr = undefined_flags});
  std::string error;
  try {
    fixture.cpu.step();
  } catch (const blofeld::Fault &fault) { error = fault.what(); }
  require_cycle(error.contains("HC12 DAA pc=0x8000") &&
                    error.contains("Unqualified DAA operand combination A=0xff C=1 H=1"),
                "DAA rejects inputs outside the documented valid post-addition combinations");
}

}

int main() {
  try {
    test_move_modes();
    test_modify_and_test_modes();
    test_lea_and_relative_call();
    test_branch_conditions();
    test_arithmetic_cycles();
    test_decimal_addition();
    test_index_counters();
    test_ccr_stack();
    test_pc_relative_word_operands();
    test_unqualified_operands_reject();
    std::cout
        << "Native cycle/state vectors, branch truth tables and 20000 decimal additions passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
