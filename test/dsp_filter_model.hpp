// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace blofeld::test::filters {
// Independent integer oracle adapted from blofeld-src ee53815320ca32fe5bc4417a60e469dd68349db1.
// This test model never supplies results to the emulator.
inline constexpr unsigned block_length = 16;
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr std::int64_t word_unit = std::int64_t{1} << 24;
inline constexpr std::int64_t half = word_unit / 2;
inline constexpr std::int64_t segment_width = std::int64_t{1} << 17;
inline constexpr unsigned rounding_mode = 1U << 21;
inline constexpr unsigned table_base = 0x91d;
inline constexpr unsigned curve_base = 0xa34;
inline constexpr unsigned table_words = 603;
using Block = std::array<unsigned, block_length + 1>;
using History = std::array<unsigned, 4>; // Older/newer input, older/newer output.
enum class Kind : std::uint8_t { low, band, high, notch };

struct Input {
  Kind kind = Kind::low;
  std::int64_t cutoff = 0;
  std::int64_t resonance = 0;
  bool twos = false;
  unsigned pattern = 0;
  bool cascade = false;
};

struct Coefficients {
  std::int64_t b0 = 0;
  std::int64_t b1 = 0;
  std::int64_t b2 = 0;
  std::int64_t a1 = 0;
  std::int64_t a2 = 0;
  std::int64_t g = 0;
  std::int64_t interpolant = 0;
  bool wide = false;
  bool clipped = false;
  bool damping_halved = false;
  bool cap_selected = false;
};

inline constexpr History initial_history{0x123456, 0xedcbaa, 0x234567, 0xdcba99};
inline constexpr History initial_second_history{0x345678, 0xcba988, 0x456789, 0xba9877};

struct Prediction {
  Coefficients final;
  Coefficients first;
  Block input{};
  Block intermediate{};
  Block output{};
  History first_history = initial_history;
  History second_history = initial_second_history;
  std::string context;
};

std::int64_t signed_word(unsigned value);
unsigned raw(std::int64_t value);
Prediction predict(const Input &input, std::span<const unsigned> table);
}
