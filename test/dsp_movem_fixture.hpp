// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld::test::movem {
constexpr unsigned register_count = 46, ram = 0x100, start = 0x1fa, entry = 0x200;
constexpr unsigned data = 0x600, short_data = 0x3c, patch = 0x209, callee = 0x260;
constexpr unsigned ordinary_omr = 0x300, stage_omr = 0x600389, seed_words = 54;
constexpr unsigned seed_instructions = 32, initial_cycles = 60, maximum_points = 2048;
constexpr unsigned move_omr = 0x05f43a, move_sz = 0x05f438, value_mask = 0xffffff;
constexpr unsigned pc_field = 10, sr_field = 11, omr_field = 12, la_field = 13, lc_field = 14;
constexpr unsigned sp_field = 15, sc_field = 18, first_r = 20, first_m = 36, ssh = 44, ssl = 45;
constexpr unsigned x0_field = 1, x1_field = 0, a1_field = 5, b1_field = 8;
using Registers = std::array<unsigned, register_count>;
using StackWords = std::array<unsigned, 32>;

enum class Kind : std::uint8_t {
  load,
  store,
  absolute_load,
  absolute_store,
  short_load,
  short_store,
  loop,
  patch_loop
};
enum class Producer : std::uint8_t { none, alu, address };

struct Case {
  Kind kind;
  unsigned reg = 7, omr = ordinary_omr, gap = 0;
  Producer producer = Producer::none;
  bool call = false;
};

struct Cell {
  DspSpace space;
  unsigned address, value;
};

struct Point {
  unsigned pc = 0, opcode = 0, cost = 1, words = 1, extension = 0, kind = 0;
  DspSpace memory_space = DspSpace::p;
  unsigned address = 0, value = 0;
};

struct Row {
  Point point;
  Registers state;
  StackWords stack;
  std::vector<unsigned> memory;
};

enum class Effect : std::uint8_t { nop, load, store, alu_a, alu_b_r3, r7, loop, call, ret };

struct Instruction {
  unsigned opcode = 0, cost = 1, words = 1, extension = 0;
  Effect effect = Effect::nop;
  unsigned field = 0, reg = 0, address = 0;
  bool absolute = false, post = false;
};

struct Program {
  std::vector<unsigned> words;
  std::vector<Instruction> instructions;
  std::vector<Cell> cells;
  unsigned stop = 0;
};

struct Window {
  Program program;
  Registers initial;
  std::vector<Row> rows;
  unsigned periods;
};

std::vector<Case> cases();
std::string name(Case sample);
Registers initial(Case sample);
Program program(Case sample);
Instruction program_move(bool load,
                         unsigned operand,
                         unsigned reg,
                         bool post = false,
                         bool absolute = false,
                         bool short_address = false);
unsigned periods(Case sample);
Window window(Case sample);
}
