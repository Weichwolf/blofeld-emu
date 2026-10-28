// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp_movem_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace blofeld::test::movem {
namespace {
constexpr unsigned image_words = 512;
constexpr unsigned tail = 7;
constexpr unsigned padding = 8;
constexpr unsigned branch_low_mask = 31;
constexpr unsigned cell_capacity = 18;
constexpr unsigned tfr_b_r3 = 0x21b359;
constexpr unsigned old_x_store = 0x555b00;
constexpr unsigned tfr_a = 0x200051;
constexpr unsigned r7_immediate = 0x67f400;
constexpr unsigned short_prefix = 0x070000;
constexpr unsigned effective_prefix = 0x074080;
constexpr unsigned branch_prefix = 0x050800;
constexpr unsigned branch_modulus = 512;
constexpr unsigned seed_sr = 0x05f439;
constexpr unsigned seed_la = 0x05f43e;
constexpr unsigned seed_lc = 0x05f43f;
constexpr unsigned loop_opcode = 0x06c710;
constexpr unsigned patch_pointer = 0x610;

unsigned put(Program &image, const unsigned pc, const Instruction instruction) {
  image.words.at(pc - ram) = instruction.opcode;
  image.instructions.at(pc - ram) = instruction;
  if (instruction.words == 2) { image.words.at(pc + 1 - ram) = instruction.extension; }
  return pc + instruction.words;
}

void seeds(Program &image, const Case sample) {
  const auto values = initial(sample);
  constexpr std::array fields{omr_field,
                              17U,
                              sr_field,
                              la_field,
                              lc_field,
                              x0_field,
                              x1_field,
                              3U,
                              2U,
                              6U,
                              a1_field,
                              4U,
                              9U,
                              b1_field,
                              7U};
  constexpr std::array opcodes{move_omr,
                               move_sz,
                               seed_sr,
                               seed_la,
                               seed_lc,
                               0x44f400U,
                               0x45f400U,
                               0x46f400U,
                               0x47f400U,
                               0x50f400U,
                               0x54f400U,
                               0x52f400U,
                               0x51f400U,
                               0x55f400U,
                               0x53f400U};
  unsigned pc = ram;
  for (std::size_t index = 0; index < fields.size(); ++index) {
    pc = put(image,
             pc,
             {.opcode = opcodes.at(index),
              .cost = 2,
              .words = 2,
              .extension = values.at(fields.at(index))});
  }
  constexpr unsigned r_immediate = 0x60f400;
  for (unsigned reg = 0; reg < 8; ++reg) {
    pc = put(image,
             pc,
             {.opcode = r_immediate | (reg << 16U), .cost = 2, .words = 2, .extension = data});
  }
  constexpr unsigned guards = 6;
  constexpr unsigned jump = 0x0af080;
  static_cast<void>(
      put(image, pc + guards, {.opcode = jump, .cost = 4, .words = 2, .extension = start}));
}

Instruction call(const unsigned pc) {
  const auto delta = (callee + branch_modulus - pc) & (branch_modulus - 1);
  return {.opcode = branch_prefix | ((delta >> 5U) << 6U) | (delta & branch_low_mask),
          .cost = 4,
          .effect = Effect::call,
          .address = callee};
}

Instruction loop(const unsigned pc) {
  return {.opcode = loop_opcode,
          .cost = 5,
          .words = 2,
          .extension = 3,
          .effect = Effect::loop,
          .address = pc + 3};
}

unsigned patch_body(Program &image) {
  unsigned pc = put(image, entry, program_move(true, 5, 1));
  pc = put(image, pc, {.opcode = tfr_b_r3, .effect = Effect::alu_b_r3});
  auto store = program_move(false, 5, 0, false, true);
  store.address = patch;
  store.extension = patch;
  pc = put(image, pc, store);
  pc += 2;
  pc = put(image, pc, loop(pc));
  pc = put(image, pc, call(pc));
  return put(
      image,
      pc,
      {.opcode = old_x_store, .effect = Effect::store, .field = b1_field, .reg = 3, .post = true});
}

unsigned body(Program &image, const Case sample) {
  if (sample.kind == Kind::patch_loop) { return patch_body(image); }
  unsigned pc = entry;
  if (sample.producer == Producer::alu) {
    pc = put(image, pc, {.opcode = tfr_a, .effect = Effect::alu_a});
  }
  if (sample.producer == Producer::address) {
    pc = put(image,
             pc,
             {.opcode = r7_immediate,
              .cost = 2,
              .words = 2,
              .extension = data + 4,
              .effect = Effect::r7});
  }
  pc += sample.gap;
  if (sample.kind == Kind::loop) {
    pc = put(image, pc, loop(pc));
    pc = put(image, pc, sample.call ? call(pc) : Instruction{});
    return put(image, pc, program_move(false, 12, sample.reg, true));
  }
  const bool load = sample.kind == Kind::load || sample.kind == Kind::absolute_load ||
                    sample.kind == Kind::short_load;
  const bool absolute = sample.kind == Kind::absolute_load || sample.kind == Kind::absolute_store;
  const bool short_address = sample.kind == Kind::short_load || sample.kind == Kind::short_store;
  return put(image,
             pc,
             program_move(load,
                          load || absolute || short_address ? 4U : 12U,
                          sample.reg,
                          !load && !absolute && !short_address,
                          absolute || short_address,
                          short_address));
}

void cells(Program &image, const Case sample) {
  constexpr unsigned sentinel = 0x102030;
  constexpr unsigned increment = 0x010101;
  constexpr unsigned short_value = 0xabcdef;
  image.cells.reserve(cell_capacity);
  for (unsigned index = 0; index < 8; ++index) {
    image.cells.push_back(
        {.space = DspSpace::p, .address = data + index, .value = sentinel + (index * increment)});
  }
  image.cells.push_back({.space = DspSpace::p, .address = short_data, .value = short_value});
  if (sample.kind == Kind::patch_loop) {
    image.cells.front().value = program_move(false, 13, 3, true).opcode;
    image.cells.push_back(
        {.space = DspSpace::p, .address = patch, .value = image.words.at(patch - ram)});
    constexpr unsigned p_sentinel = 0x223344;
    constexpr unsigned x_sentinel = 0x334455;
    for (unsigned index = 0; index < 4; ++index) {
      image.cells.push_back(
          {.space = DspSpace::p, .address = patch_pointer + index, .value = p_sentinel + index});
      image.cells.push_back(
          {.space = DspSpace::x, .address = patch_pointer + index, .value = x_sentinel + index});
    }
  }
  std::ranges::sort(
      image.cells, {}, [](const Cell cell) { return std::pair{cell.space, cell.address}; });
}
}

Instruction program_move(const bool load,
                         const unsigned operand,
                         const unsigned reg,
                         const bool post,
                         const bool absolute,
                         const bool short_address) {
  constexpr unsigned x0 = 4;
  constexpr unsigned x1 = 5;
  constexpr unsigned a1 = 12;
  constexpr unsigned b1 = 13;
  unsigned field = 0;
  switch (operand) {
    case x0: field = x0_field; break;
    case x1: field = x1_field; break;
    case a1: field = a1_field; break;
    case b1: field = b1_field; break;
    default: throw std::logic_error("Undefined own MOVEM raw data field");
  }
  unsigned mode = post ? 3U : 4U;
  if (absolute) { mode = 6; }
  const unsigned address = short_address ? short_data : data;
  const auto encoding = short_address
                            ? short_prefix | (short_data << 8U)
                            : effective_prefix | (mode << 11U) | ((absolute ? 0U : reg) << 8U);
  return {.opcode = encoding | (static_cast<unsigned>(load) << 15U) | operand,
          .cost = absolute && !short_address ? 7U : 6U,
          .words = absolute && !short_address ? 2U : 1U,
          .extension = absolute && !short_address ? address : 0,
          .effect = load ? Effect::load : Effect::store,
          .field = field,
          .reg = reg,
          .address = address,
          .absolute = absolute,
          .post = post};
}

Program program(const Case sample) {
  Program image{.words = std::vector<unsigned>(image_words),
                .instructions = std::vector<Instruction>(image_words),
                .cells = {}};
  seeds(image, sample);
  image.stop = body(image, sample) + tail;
  unsigned end = image.stop;
  if (sample.call) {
    constexpr unsigned rts = 0x00000c;
    static_cast<void>(put(image, callee + 3, {.opcode = rts, .cost = 3, .effect = Effect::ret}));
    end = callee + 4;
  }
  image.words.resize(end - ram + padding);
  image.instructions.resize(image.words.size());
  cells(image, sample);
  return image;
}
}
