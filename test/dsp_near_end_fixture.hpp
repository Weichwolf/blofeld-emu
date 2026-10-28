// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace blofeld::test::near_end {
constexpr unsigned ram = 0x100, start = 0x1fa, entry = 0x200, body = 0x202;
constexpr unsigned callee = 0x240, helper = 0x280, image_words = 0x200;
constexpr unsigned seed_words = 26, seed_instructions = 18, first_e2 = 7;
constexpr unsigned maximum_periods = 20000, tail_words = 7, padding = 8;
constexpr unsigned move_omr = 0x05f43a, move_sz = 0x05f438, jump = 0x0af080;
constexpr unsigned dor_y1 = 0x06c710, dor_immediate = 0x060090, rts = 0x00000c;
constexpr unsigned short_bsr = 0x050800, andi_ccr = 0x00ffb9, flags = 0xff;
constexpr unsigned short_modulus = 512, low_mask = 31, split_shift = 5, opcode_shift = 6;
constexpr unsigned la_seed = 0x55aa33, lc_seed = 0x33, ordinary_omr = 0x300, stage_omr = 0x600389;
constexpr unsigned x0_seed = 0x112233, x1_seed = 0x445566, y0_seed = 0x778899;
constexpr unsigned loop_flags = 0x18000, loop_flag = 0x8000, refill_count = 176;
constexpr unsigned expected_streams = 17, expected_periods = 21166;
constexpr unsigned expected_snapshot_cases = 10, expected_cuts = 1916;

struct Case {
  std::string_view name;
  unsigned size = 2, count = 3, inner = 0, omr = ordinary_omr;
  bool adjacent = false, joined = false;
};

enum class Effect : std::uint8_t { nop, loop, call, ret, flags };

struct Instruction {
  unsigned opcode = 0, cost = 1, words = 1;
  Effect effect = Effect::nop;
  unsigned target = 0, last = 0, count = 0;
};

struct Program {
  std::vector<unsigned> words = std::vector<unsigned>(image_words, 0);
  std::vector<Instruction> instructions = std::vector<Instruction>(image_words);
  unsigned stop = 0;
};

inline unsigned put(Program &code, const unsigned pc, const Instruction instruction) {
  code.words.at(pc - ram) = instruction.opcode;
  code.instructions.at(pc - ram) = instruction;
  return pc + instruction.words;
}

struct Call {
  unsigned address;
  unsigned destination;
};

inline void call(Program &code, const Call branch) {
  const auto delta = (branch.destination + short_modulus - branch.address) & (short_modulus - 1);
  static_cast<void>(
      put(code,
          branch.address,
          {.opcode = short_bsr | ((delta >> split_shift) << opcode_shift) | (delta & low_mask),
           .cost = 4,
           .effect = Effect::call,
           .target = branch.destination}));
}

struct Loop {
  unsigned address;
  unsigned count;
  unsigned size;
  bool immediate;
};

inline void loop(Program &code, const Loop frame) {
  const auto last = frame.address + 1 + frame.size;
  const auto opcode = frame.immediate
                          ? dor_immediate | ((frame.count & flags) << 8U) | (frame.count >> 8U)
                          : dor_y1;
  static_cast<void>(put(code,
                        frame.address,
                        {.opcode = opcode,
                         .cost = 5,
                         .words = 2,
                         .effect = Effect::loop,
                         .target = frame.address + 2,
                         .last = last,
                         .count = frame.count}));
  code.words.at(frame.address + 1 - ram) = last - frame.address;
}

inline void seeds(Program &code, const Case sample) {
  const std::array initial{move_omr,
                           sample.omr,
                           move_sz,
                           0U,
                           0x05f439U,
                           masked_status | flags,
                           0x05f43eU,
                           la_seed,
                           0x05f43fU,
                           lc_seed,
                           0x44f400U,
                           x0_seed,
                           0x45f400U,
                           x1_seed,
                           0x46f400U,
                           y0_seed,
                           0x47f400U,
                           sample.count};
  for (std::size_t index = 0; index < initial.size(); ++index) {
    code.words.at(index) = initial[index];
  }
  constexpr unsigned guards = 6;
  code.words.at(initial.size() + guards) = jump;
  code.words.at(initial.size() + guards + 1) = start;
}

inline unsigned callees(Program &code, const Case sample) {
  unsigned pc = callee;
  if (sample.joined) {
    loop(code, {.address = callee, .count = sample.inner, .size = 8, .immediate = true});
    for (const auto offset : {0U, 2U, 5U}) {
      call(code, {.address = callee + 2 + offset, .destination = helper});
    }
    static_cast<void>(put(code, callee + 10, {.opcode = rts, .cost = 3, .effect = Effect::ret}));
    pc = helper + 3;
  } else {
    pc += 3;
    if (sample.inner != 0) {
      loop(code, {.address = pc, .count = sample.inner, .size = 3, .immediate = true});
      pc += 5;
    }
    if (sample.adjacent) {
      static_cast<void>(put(code, pc, {.opcode = rts, .cost = 3, .effect = Effect::ret}));
      return pc + 5; // Retain the independent image's unused trailing words.
    }
  }
  pc = put(code, pc, {.opcode = andi_ccr, .cost = 3, .effect = Effect::flags}) + 3;
  return put(code, pc, {.opcode = rts, .cost = 3, .effect = Effect::ret});
}

inline Program program(const Case sample) {
  Program code;
  seeds(code, sample);
  loop(code,
       {.address = entry,
        .count = sample.count,
        .size = sample.size,
        .immediate = sample.size == 8});
  for (const auto offset : {0U, 2U, 5U}) {
    if (offset == 0 || sample.size == 8) {
      call(code, {.address = body + offset, .destination = callee});
    }
  }
  code.stop = body + sample.size + tail_words;
  const auto end = callees(code, sample);
  code.words.resize(end - ram + padding);
  code.instructions.resize(code.words.size());
  return code;
}

inline DspDebugState initial(const Case sample) {
  DspDebugState expected;
  expected.cpu.pc = start;
  expected.cpu.omr = sample.omr;
  expected.cpu.sr = masked_status | flags;
  expected.cpu.la = la_seed;
  expected.cpu.lc = lc_seed;
  expected.cpu.x = (std::uint64_t{x1_seed} << word_bits) | x0_seed;
  expected.cpu.y = (std::uint64_t{sample.count} << word_bits) | y0_seed;
  expected.cpu.m.fill(word_mask);
  expected.native_instructions = seed_instructions;
  return expected;
}

struct LoopFrame {
  DspState restore;
  unsigned sc;
  unsigned body;
  unsigned last;
  unsigned remaining;
};

struct Oracle {
  DspDebugState expected;
  std::vector<LoopFrame> loops;
  unsigned next_e2 = first_e2;
  bool return_to_last = false;
};

struct StackWord {
  unsigned high;
  unsigned low;
};

inline void push(DspDebugState &expected, const StackWord frame) {
  ++expected.cpu.sp;
  ++expected.sc;
  expected.stack_high.at(expected.cpu.sp) = frame.high;
  expected.stack_low.at(expected.cpu.sp) = frame.low;
}

inline void close_loop(Oracle &oracle, const unsigned pc) {
  if (oracle.loops.empty() || pc != oracle.loops.back().last) { return; }
  auto &frame = oracle.loops.back();
  auto &cpu = oracle.expected.cpu;
  if (--frame.remaining != 0) {
    cpu.pc = frame.body;
    cpu.lc = frame.remaining;
    return;
  }
  cpu.la = frame.restore.la;
  cpu.lc = frame.restore.lc;
  cpu.sp = frame.restore.sp;
  oracle.expected.sc = frame.sc;
  cpu.sr = (cpu.sr & ~loop_flags) | (frame.restore.sr & loop_flags);
  oracle.loops.pop_back();
}

inline void retire(Oracle &oracle, const Program &code) {
  auto &expected = oracle.expected;
  auto &cpu = expected.cpu;
  const auto pc = cpu.pc;
  const auto instruction = code.instructions.at(pc - ram);
  const bool extra = oracle.return_to_last;
  oracle.return_to_last = false;
  cpu.pc += instruction.words;
  switch (instruction.effect) {
    case Effect::loop:
      oracle.loops.push_back({.restore = cpu,
                              .sc = expected.sc,
                              .body = instruction.target,
                              .last = instruction.last,
                              .remaining = instruction.count});
      push(expected, {.high = cpu.la, .low = cpu.lc});
      push(expected, {.high = instruction.target, .low = cpu.sr});
      cpu.pc = instruction.target;
      cpu.la = instruction.last;
      cpu.lc = instruction.count;
      cpu.sr = (cpu.sr & ~loop_flags) | loop_flag;
      break;
    case Effect::call:
      push(expected, {.high = cpu.pc, .low = cpu.sr});
      cpu.pc = instruction.target;
      break;
    case Effect::ret:
      cpu.pc = expected.stack_high.at(cpu.sp);
      --cpu.sp;
      --expected.sc;
      oracle.return_to_last = cpu.pc == cpu.la;
      break;
    case Effect::flags: // ANDIff preserves every SR bit.
    case Effect::nop: break;
    default: throw Fault("Unknown own near-end instruction effect");
  }
  close_loop(oracle, pc);
  cpu.stack_pc = expected.stack_high.at(cpu.sp);
  cpu.stack_sr = expected.stack_low.at(cpu.sp);
  ++expected.native_instructions;
  expected.native_cycles += instruction.cost;
  oracle.next_e2 += instruction.cost + (extra ? 1U : 0U);
}

inline std::vector<Case> cases() {
  return {
      {.name = "andi-tail1-nested0-omr300"},
      {.name = "andi-tail2-nested0-omr300", .size = 3},
      {.name = "andi-tail1-nested1-omr300", .size = 2, .count = 3, .inner = 2},
      {.name = "andi-refill-three-calls-nested0-omr300", .size = 8},
      {.name = "andi-tail1-nested0-omr600389", .size = 2, .count = 3, .inner = 0, .omr = stage_omr},
      {.name = "followup-tail1-count1-adjacent0-omr300", .size = 2, .count = 1},
      {.name = "followup-tail2-count1-adjacent0-omr300", .size = 3, .count = 1},
      {.name = "followup-tail1-count1-adjacent1-omr300",
       .size = 2,
       .count = 1,
       .inner = 2,
       .omr = ordinary_omr,
       .adjacent = true},
      {.name = "followup-tail1-count3-adjacent1-omr300",
       .size = 2,
       .count = 3,
       .inner = 2,
       .omr = ordinary_omr,
       .adjacent = true},
      {.name = "followup-tail2-count1-adjacent1-omr300",
       .size = 3,
       .count = 1,
       .inner = 2,
       .omr = ordinary_omr,
       .adjacent = true},
      {.name = "followup-tail2-count3-adjacent1-omr300",
       .size = 3,
       .count = 3,
       .inner = 2,
       .omr = ordinary_omr,
       .adjacent = true},
      {.name = "followup-refill-three-calls-count3-adjacent1-omr300",
       .size = 8,
       .count = 3,
       .inner = 2,
       .omr = ordinary_omr,
       .adjacent = true},
      {.name = "followup-refill-three-calls-count3-adjacent1-omr600389",
       .size = 8,
       .count = 3,
       .inner = 2,
       .omr = stage_omr,
       .adjacent = true},
      {.name = "joined-outer3-inner3-omr300",
       .size = 2,
       .count = 3,
       .inner = 3,
       .omr = ordinary_omr,
       .adjacent = true,
       .joined = true},
      {.name = "joined-outer3-inner3-omr600389",
       .size = 2,
       .count = 3,
       .inner = 3,
       .omr = stage_omr,
       .adjacent = true,
       .joined = true},
      {.name = "joined-outer1-inner176-omr300",
       .size = 2,
       .count = 1,
       .inner = refill_count,
       .omr = ordinary_omr,
       .adjacent = true,
       .joined = true},
      {.name = "joined-outer1-inner176-omr600389",
       .size = 2,
       .count = 1,
       .inner = refill_count,
       .omr = stage_omr,
       .adjacent = true,
       .joined = true}};
}

inline bool snapshot_case(const Case sample) {
  return sample.name.starts_with("followup-") || (sample.joined && sample.count == 3);
}
}
