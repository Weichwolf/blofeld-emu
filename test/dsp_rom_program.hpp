// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <vector>

namespace blofeld::test {
enum class DspRomRegister : std::uint8_t {
  x0 = 4,
  x1 = 5,
  y0 = 6,
  y1 = 7,
  a0 = 8,
  b0 = 9,
  a2 = 10,
  b2 = 11,
  a1 = 12,
  b1 = 13,
  a = 14,
  b = 15,
  r0 = 16,
  r1 = 17,
  r2 = 18,
  r3 = 19,
  r4 = 20,
  r5 = 21,
  r6 = 22,
  r7 = 23,
  n0 = 24,
  n1 = 25,
  n2 = 26,
  n3 = 27,
  n4 = 28,
  n5 = 29,
  n6 = 30,
  n7 = 31,
  m0 = 32,
  m1 = 33,
  m2 = 34,
  m3 = 35,
  m4 = 36,
  m5 = 37,
  m6 = 38,
  m7 = 39,
  sr = 57,
  omr = 58,
};

// Authored fixture words, not an assembler or a processor implementation.
class DspRomProgram {
public:
  static constexpr std::uint32_t boot = 0xff0000;
  static constexpr std::uint32_t word_mask = 0xffffff;
  static constexpr std::size_t maximum_words = 65536;

  void append(const std::span<const std::uint32_t> words) {
    if (words.size() > maximum_words - words_.size() ||
        std::ranges::any_of(words, [](const auto word) { return word > word_mask; })) {
      throw Fault("DSP ROM fixture exceeds its word or image bounds");
    }
    words_.insert(words_.end(), words.begin(), words.end());
  }

  void word(const std::uint32_t value) { append(std::array{value}); }

  // FM table12-14 and MOVE/MOVEC immediate fields. This fixture subset covers
  // scalar data/AGU/M registers plus SR/OMR, with an explicit full-word literal.
  void immediate(const DspRomRegister reg, const std::uint32_t value) {
    const auto selector = static_cast<unsigned>(reg);
    constexpr unsigned first_data = 4;
    constexpr unsigned last_modifier = 39;
    constexpr unsigned status = 57;
    constexpr unsigned mode = 58;
    const bool scalar = selector >= first_data && selector <= last_modifier;
    if (!scalar && selector != status && selector != mode) {
      throw Fault("Unsupported DSP ROM fixture immediate register");
    }
    constexpr unsigned first_control = 32;
    constexpr unsigned low_register_mask = 7;
    constexpr unsigned upper_register_mask = 24;
    constexpr unsigned low_shift = 16;
    constexpr unsigned upper_shift = 17;
    constexpr unsigned control_opcode = 0x05f400;
    constexpr unsigned scalar_opcode = 0x40f400;
    const auto opcode = selector >= first_control
                            ? control_opcode | selector
                            : scalar_opcode | ((selector & low_register_mask) << low_shift) |
                                  ((selector & upper_register_mask) << upper_shift);
    append(std::array{opcode, value});
  }

  [[nodiscard]] std::uint32_t pc() const {
    return boot + static_cast<std::uint32_t>(words_.size());
  }

  [[nodiscard]] std::span<const std::uint32_t> words() const { return words_; }

private:
  std::vector<std::uint32_t> words_;
};

inline void write_dsp_rom(const std::filesystem::path &path, const DspRomProgram &program) {
  std::ofstream stream(path, std::ios::binary);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  for (const auto word : program.words()) {
    constexpr unsigned high_shift = 16;
    constexpr unsigned low_shift = 8;
    constexpr unsigned byte_mask = 0xff;
    const std::array bytes{static_cast<char>((word >> high_shift) & byte_mask),
                           static_cast<char>((word >> low_shift) & byte_mask),
                           static_cast<char>(word & byte_mask)};
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  stream.close();
}
}
