// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld::test::parallel_middle {
constexpr unsigned ram = 0x100, start = 0x1fa, entry = 0x200, image_words = 0x200;
constexpr unsigned first_e2 = 7, tail_words = 7, maximum_periods = 64;
constexpr unsigned seed_words = 34, seed_instructions = 22, expected_streams = 180;
constexpr unsigned expected_periods = 3710, scale_shift = 10;
constexpr unsigned expected_snapshot_cases = 24, expected_cuts = 532;
constexpr unsigned omr = 0x300, status = 0xc00300, flags_seed = 0xaa;
constexpr unsigned la_seed = 0x55aa33, lc_seed = 0x33;
constexpr unsigned x0_seed = 3, x1_seed = 0x445566, y0_seed = 0x91abcd, y1_seed = 0xaabbcc;
constexpr unsigned a0_code = 8, a2_code = 10, a1_code = 12, b1_code = 13, r0_code = 16;
constexpr unsigned byte_mask = 0xff, mover = 0x200000, source_shift = 13, target_shift = 8;
constexpr unsigned move_omr = 0x05f43a, move_sz = 0x05f438, jump = 0x0af080;
constexpr std::uint64_t negative_extension = 0xff000000000000;
constexpr unsigned fraction_sign = 0x800000, positive_overflow_vector = 3, zero_vector = 5;
constexpr std::array destinations{4U, 5U, 6U, 7U, 16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U};

enum class Alu : std::uint8_t { transfer_a, transfer_b, add_a, add_b };
enum class Effect : std::uint8_t { nop, producer, parallel };

struct Case {
  Alu alu = Alu::transfer_a;
  unsigned source = a1_code, destination = 4, flags = flags_seed;
  unsigned scale = 0, writer = 0, gap = 0, vector = 0;
  bool sequence = false;

  [[nodiscard]] bool interlock() const {
    return writer != 0 && gap == 0 && (writer & 1U) == (source & 1U);
  }
};

inline bool addition(const Alu alu) {
  return alu == Alu::add_a || alu == Alu::add_b;
}

inline bool target_b(const Alu alu) {
  return alu == Alu::transfer_b || alu == Alu::add_b;
}

// FM13-7/178,Tables12-13/16: independent literal ALU bytes and raw move fields.
inline unsigned opcode(const Case sample) {
  constexpr std::array alu_bytes{0x51U, 0x59U, 0x10U, 0x18U};
  return mover | (sample.source << source_shift) | (sample.destination << target_shift) |
         alu_bytes.at(static_cast<unsigned>(sample.alu));
}

inline std::array<std::uint64_t, 2> accumulators(const unsigned vector) {
  constexpr std::array values{
      std::array{std::uint64_t{0xe9812345d4e5f6}, std::uint64_t{0x125577996c7d8e}},
      std::array{std::uint64_t{0x00100000000000}, std::uint64_t{0x00200000000000}},
      std::array{accumulator_mask, std::uint64_t{1}},
      std::array{sign_bit - 1, std::uint64_t{1}},
      std::array{sign_bit, accumulator_mask},
      std::array{std::uint64_t{0x00100000000000}, std::uint64_t{0xfff00000000000}},
      std::array{std::uint64_t{0xff900000000000}, std::uint64_t{0xff900000000000}}};
  return values.at(vector);
}

struct Instruction {
  unsigned opcode = 0, target = 0, source = 0, hold_before = 0;
  Effect effect = Effect::nop;
  Alu alu = Alu::transfer_a;
};

struct Program {
  std::vector<unsigned> words = std::vector<unsigned>(image_words, 0);
  std::vector<Instruction> instructions = std::vector<Instruction>(image_words);
  unsigned stop = 0;
};

inline void seeds(Program &code, const Case sample) {
  const auto values = accumulators(sample.vector);
  const auto part = [](const std::uint64_t value, const unsigned index) {
    return static_cast<unsigned>((value >> (index * word_bits)) & word_mask);
  };
  const std::array initial{0x05f439U, status | sample.flags | (sample.scale << scale_shift),
                           0x05f43eU, la_seed,
                           0x05f43fU, lc_seed,
                           0x44f400U, x0_seed,
                           0x45f400U, x1_seed,
                           0x46f400U, y0_seed,
                           0x47f400U, y1_seed,
                           0x50f400U, part(values[0], 0),
                           0x54f400U, part(values[0], 1),
                           0x52f400U, part(values[0], 2),
                           0x51f400U, part(values[1], 0),
                           0x55f400U, part(values[1], 1),
                           0x53f400U, part(values[1], 2)};
  for (std::size_t index = 0; index < initial.size(); ++index) {
    code.words.at(index) = initial[index];
  }
  constexpr unsigned separation = 6;
  code.words.at(initial.size() + separation) = jump;
  code.words.at(initial.size() + separation + 1) = start;
}

inline unsigned put(Program &code, const unsigned pc, const Instruction instruction) {
  code.words.at(pc - ram) = instruction.opcode;
  code.instructions.at(pc - ram) = instruction;
  return pc + 1;
}

inline unsigned parallel(Program &code, const unsigned pc, const Case sample) {
  return put(code,
             pc,
             {.opcode = opcode(sample),
              .target = sample.destination,
              .source = sample.source,
              .hold_before = sample.interlock() ? 1U : 0U,
              .effect = Effect::parallel,
              .alu = sample.alu});
}

inline Program program(const Case sample) {
  Program code;
  seeds(code, sample);
  auto pc = entry;
  if (sample.writer != 0) {
    pc = put(code,
             pc,
             {.opcode = mover | (4U << source_shift) | (sample.writer << target_shift),
              .target = sample.writer,
              .effect = Effect::producer}) +
         sample.gap;
  }
  pc = parallel(code, pc, sample);
  if (sample.sequence) {
    pc = parallel(code,
                  pc,
                  {.alu = sample.source == a1_code ? Alu::transfer_a : Alu::transfer_b,
                   .source = sample.source,
                   .destination = r0_code + 7});
  }
  code.stop = pc + tail_words;
  // Entire own RAM image is installed,including actual NOP words for F2 prefetch.
  return code;
}

inline void write_word(DspState &cpu, const unsigned reg, const unsigned value) {
  if (reg < a0_code) {
    auto &pair = reg < 6 ? cpu.x : cpu.y;
    const auto shift = (reg & 1U) * word_bits;
    pair = (pair & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
  } else if (reg < r0_code) {
    auto &accumulator = (reg & 1U) == 0 ? cpu.a : cpu.b;
    unsigned part = 1;
    if (reg < a2_code) {
      part = 0;
    } else if (reg < a1_code) {
      part = 2;
    }
    const auto shift = part * word_bits;
    const std::uint64_t mask = part == 2 ? byte_mask : word_mask;
    accumulator = (accumulator & ~(mask << shift)) | (std::uint64_t{value} << shift);
  } else {
    cpu.r.at(reg - r0_code) = value;
  }
}

inline DspMoveFields fields(const unsigned reg) {
  DspMoveFields result;
  if (reg < a0_code) {
    auto &pair = reg < 6 ? result.x : result.y;
    pair = static_cast<std::uint8_t>(1U << (reg & 1U));
  } else if (reg < r0_code) {
    auto &accumulator = (reg & 1U) == 0 ? result.a : result.b;
    unsigned part = 1;
    if (reg < a2_code) {
      part = 0;
    } else if (reg < a1_code) {
      part = 2;
    }
    accumulator = static_cast<std::uint8_t>(1U << part);
  } else {
    result.r = static_cast<std::uint8_t>(1U << (reg - r0_code));
  }
  return result;
}

struct Arithmetic {
  std::uint64_t value;
  unsigned status;
  bool b;
};

// Independent FM Table5-1 signed56-bit sum and E/U/N/Z definitions.
inline Arithmetic arithmetic(const DspState &cpu, const Alu alu) {
  Arithmetic result{.value = 0, .status = cpu.sr, .b = target_b(alu)};
  if (!addition(alu)) {
    const auto y0 = cpu.y & word_mask;
    result.value = (y0 << word_bits) | ((y0 & fraction_sign) != 0 ? negative_extension : 0);
    return result;
  }
  const auto sum = cpu.a + cpu.b;
  result.value = sum & accumulator_mask;
  const bool over = (~(cpu.a ^ cpu.b) & (cpu.a ^ result.value) & sign_bit) != 0;
  result.status = (cpu.sr & ~(carry | overflow | extension | unnormalized | negative | zero)) |
                  value_flags({.value = result.value, .status = cpu.sr});
  if (sum > accumulator_mask) { result.status |= carry; }
  if (over) { result.status |= overflow | limit; }
  return result;
}

inline void append_destinations(std::vector<Case> &result) {
  for (const auto alu : {Alu::transfer_a, Alu::transfer_b, Alu::add_a, Alu::add_b}) {
    for (const auto source : {a1_code, b1_code}) {
      for (const auto destination : destinations) {
        result.push_back({.alu = alu, .source = source, .destination = destination});
      }
    }
  }
}

inline void append_flags(std::vector<Case> &result) {
  constexpr unsigned last_vector = 6;
  for (const auto alu : {Alu::add_a, Alu::add_b}) {
    for (unsigned vector = 1; vector <= last_vector; ++vector) {
      for (const auto flags : {0U, byte_mask}) {
        result.push_back(
            {.alu = alu, .destination = r0_code + 7, .flags = flags, .vector = vector});
      }
    }
  }
}

inline void append_producers(std::vector<Case> &result) {
  for (const auto low : {0U, 1U}) {
    const auto alu = low == 0 ? Alu::transfer_a : Alu::transfer_b;
    const auto add = low == 0 ? Alu::add_a : Alu::add_b;
    for (const auto scale : {1U, 2U, 3U}) {
      result.push_back({.alu = alu, .source = a1_code + low, .destination = 7, .scale = scale});
      if (scale != 3) {
        result.push_back({.alu = add, .source = a1_code + low, .destination = 7, .scale = scale});
      }
    }
    for (const auto writer : {a0_code + low, a1_code + low, a2_code + low, a1_code + (1U - low)}) {
      for (const auto gap : {0U, 1U, 2U}) {
        result.push_back({.alu = alu,
                          .source = a1_code + low,
                          .destination = r0_code + 7,
                          .writer = writer,
                          .gap = gap});
      }
    }
    result.push_back({.alu = low == 0 ? Alu::add_b : Alu::add_a,
                      .source = a1_code + low,
                      .destination = 6,
                      .sequence = true});
  }
}

inline std::vector<Case> cases() {
  std::vector<Case> result;
  append_destinations(result);
  append_flags(result);
  append_producers(result);
  const auto original = result.size();
  for (std::size_t index = 0; index < original; ++index) {
    auto sample = result.at(index);
    if (sample.writer == 0) { continue; }
    sample.alu = sample.source == a1_code ? Alu::add_b : Alu::add_a;
    result.push_back(sample);
  }
  require(result.size() == expected_streams, "All180 independently qualified input combinations");
  return result;
}

inline bool snapshot_case(const Case sample) {
  if (sample.sequence) { return true; }
  if (sample.writer != 0) {
    return sample.gap == 0 && (sample.interlock() || addition(sample.alu));
  }
  if (sample.vector != 0) {
    return sample.flags == 0 &&
           (sample.vector == positive_overflow_vector || sample.vector == zero_vector);
  }
  return sample.scale == 3 || (!addition(sample.alu) && sample.destination == 6 &&
                               target_b(sample.alu) == (sample.source == b1_code));
}
}
