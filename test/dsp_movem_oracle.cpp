// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp_movem_fixture.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace blofeld::test::movem {
namespace {
constexpr unsigned loop_bits = 0x18000, loop_flag = 0x8000, status = 0xc00300;
constexpr unsigned a2_field = 4, a0_field = 6, b2_field = 7, b0_field = 9;
constexpr unsigned r3_field = first_r + 3, r7_field = first_r + 7, middle_value = 0x778899;

struct Loop {
  Registers restore;
  unsigned body, last, remaining;
};

struct Cursor {
  Program image;
  Registers state;
  StackWords stack{};
  std::vector<Loop> loops;
};

struct StackWord {
  unsigned high, low;
};

void push(Cursor &cursor, const StackWord word) {
  const auto slot = ++cursor.state.at(sp_field);
  ++cursor.state.at(sc_field);
  cursor.stack.at(std::size_t{2} * slot) = word.high;
  cursor.stack.at((std::size_t{2} * slot) + 1) = word.low;
}

Cell &cell(Cursor &cursor, const unsigned address) {
  const auto found = std::ranges::find_if(cursor.image.cells, [address](const Cell value) {
    return value.space == DspSpace::p && value.address == address;
  });
  if (found == cursor.image.cells.end()) {
    throw std::logic_error("Undefined own expected P operand");
  }
  return *found;
}

void memory(Cursor &cursor, const Instruction instruction, const Registers &before, Point &point) {
  const auto address =
      instruction.absolute ? instruction.address : before.at(first_r + instruction.reg);
  auto &target = cell(cursor, address);
  point.kind = instruction.effect == Effect::load ? 1U : 2U;
  point.address = address;
  point.value = point.kind == 1 ? target.value : before.at(instruction.field);
  if (point.kind == 1) {
    cursor.state.at(instruction.field) = point.value;
  } else {
    target.value = point.value;
    if (address == patch) {
      const auto replacement = program_move(false, 13, 3, true);
      if (point.value != replacement.opcode) {
        throw std::logic_error("Unexpected own future-code patch");
      }
      cursor.image.words.at(patch - ram) = point.value;
      cursor.image.instructions.at(patch - ram) = replacement;
    }
  }
  if (instruction.post) { cursor.state.at(first_r + instruction.reg) = (address + 1) & value_mask; }
}

void end_loop(Cursor &cursor, const unsigned pc) {
  if (cursor.loops.empty() || cursor.loops.back().last != pc) { return; }
  auto &loop = cursor.loops.back();
  if (--loop.remaining != 0) {
    cursor.state.at(pc_field) = loop.body;
    cursor.state.at(lc_field) = loop.remaining;
    return;
  }
  for (const auto field : {la_field, lc_field, sp_field, sc_field}) {
    cursor.state.at(field) = loop.restore.at(field);
  }
  cursor.state.at(sr_field) =
      (cursor.state.at(sr_field) & ~loop_bits) | (loop.restore.at(sr_field) & loop_bits);
  cursor.loops.pop_back();
}

void effect(Cursor &cursor, const Instruction instruction, const Registers &before, Point &point) {
  auto &state = cursor.state;
  switch (instruction.effect) {
    case Effect::nop: break;
    case Effect::load:
    case Effect::store: memory(cursor, instruction, before, point); break;
    case Effect::alu_a:
      state[a2_field] = 0;
      state[a1_field] = middle_value;
      state[a0_field] = 0;
      state[sr_field] = status;
      break;
    case Effect::alu_b_r3:
      state[r3_field] = before[b1_field];
      state[b2_field] = 0;
      state[b1_field] = middle_value;
      state[b0_field] = 0;
      state[sr_field] = status;
      break;
    case Effect::r7: state[r7_field] = data + 4; break;
    case Effect::loop:
      cursor.loops.push_back(
          {.restore = before, .body = point.pc + 2, .last = instruction.address, .remaining = 3});
      push(cursor, {.high = before[la_field], .low = before[lc_field]});
      push(cursor, {.high = point.pc + 2, .low = before[sr_field]});
      state[la_field] = instruction.address;
      state[lc_field] = 3;
      state[sr_field] |= loop_flag;
      break;
    case Effect::call:
      push(cursor, {.high = state[pc_field], .low = before[sr_field]});
      state[pc_field] = instruction.address;
      break;
    case Effect::ret:
      state[pc_field] = cursor.stack.at(std::size_t{2} * state[sp_field]);
      --state[sp_field];
      --state[sc_field];
      break;
    default: throw std::logic_error("Undefined own MOVEM transition");
  }
}

Row next(Cursor &cursor) {
  const auto before = cursor.state;
  const auto pc = before.at(pc_field);
  const auto instruction = cursor.image.instructions.at(pc - ram);
  Point point{.pc = pc,
              .opcode = instruction.opcode,
              .cost = instruction.cost,
              .words = instruction.words,
              .extension = instruction.extension};
  cursor.state[pc_field] += instruction.words;
  effect(cursor, instruction, before, point);
  end_loop(cursor, pc);
  const auto slot = cursor.state.at(sp_field);
  cursor.state[ssh] = cursor.stack.at(std::size_t{2} * slot);
  cursor.state[ssl] = cursor.stack.at((std::size_t{2} * slot) + 1);
  std::vector<unsigned> values;
  values.reserve(cursor.image.cells.size());
  for (const auto value : cursor.image.cells) { values.push_back(value.value); }
  return {.point = point, .state = cursor.state, .stack = cursor.stack, .memory = values};
}
}

Window window(const Case sample) {
  Window result{.program = program(sample),
                .initial = initial(sample),
                .rows = {},
                .periods = periods(sample)};
  Cursor cursor{.image = result.program, .state = result.initial, .loops = {}};
  cursor.loops.reserve(1);
  result.rows.reserve(64);
  while (cursor.state.at(pc_field) != result.program.stop) {
    if (result.rows.size() == maximum_points) {
      throw std::logic_error("Own MOVEM oracle exceeded its finite window");
    }
    result.rows.push_back(next(cursor));
  }
  if (!cursor.loops.empty() || cursor.state.at(sp_field) != 0 || cursor.state.at(sc_field) != 0) {
    throw std::logic_error("Own MOVEM loop/call frames did not balance");
  }
  return result;
}
}
