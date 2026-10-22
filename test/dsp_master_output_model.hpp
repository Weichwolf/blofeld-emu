// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace blofeld::test::master {
inline constexpr unsigned word_bits = 24;
inline constexpr unsigned block_words = 32;
inline constexpr unsigned input_words = block_words + 1;
inline constexpr std::int64_t word_unit = std::int64_t{1} << word_bits;
inline constexpr std::int64_t minimum = -(word_unit / 2);
inline constexpr std::int64_t maximum = (word_unit / 2) - 1;
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr unsigned down = 0x400;
inline constexpr unsigned up = 0x800;
inline constexpr unsigned rounding = 0x200000;
inline constexpr unsigned entry = 0x27b;
inline constexpr unsigned end = 0x298;
inline constexpr unsigned input_base = 0x620;
inline constexpr unsigned output_base = 0x660;
inline constexpr unsigned history = 0x46;
inline constexpr unsigned guard = 0x13579b;
inline constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;

struct Input {
  unsigned volume = 0;
  unsigned scale = 0;
  unsigned phase = 0;
  unsigned pattern = 0;
  bool twos = false;
  bool dependent = false;
  bool operator==(const Input &) const = default;
};

using History = std::array<std::int64_t, 4>;

struct Samples {
  std::array<std::int64_t, input_words> mixed{};
  History history{};
};

inline Samples samples(const unsigned pattern) {
  Samples result;
  constexpr History initial{74565, -144470, 214375, -284280};
  result.history = pattern == 2 ? History{maximum, minimum, minimum, maximum} : initial;
  for (unsigned i = 0; i < result.mixed.size(); ++i) {
    switch (pattern) {
      case 0: result.mixed.at(i) = 0; break;
      case 1:
        if (i == 0) {
          result.mixed.at(i) = maximum;
        } else if (i == 1) {
          result.mixed.at(i) = minimum;
        }
        break;
      case 2: result.mixed.at(i) = i % 4 < 2 ? maximum : minimum; break;
      default: {
        constexpr std::int64_t slope = 0x13579;
        constexpr std::int64_t offset = 0x400000;
        result.mixed.at(i) = (static_cast<std::int64_t>(i) * slope) - offset;
        break;
      }
    }
  }
  return result;
}

struct Prediction {
  std::array<std::int64_t, block_words> output{};
  History history{};
  std::int64_t current = 0;
  std::int64_t previous = 0;
  std::int64_t a = 0;
  std::int64_t b = 0;
  unsigned clipped = 0;
};

struct Coefficients {
  std::int64_t current = 0;
  std::int64_t previous = 0;
  std::int64_t feedback = 0;
};

struct Transfer {
  unsigned scale = 0;
  unsigned clipped = 0;

  std::int64_t apply(const std::int64_t value) {
    unsigned shift = word_bits;
    if (scale == down) { ++shift; }
    if (scale == up) { --shift; }
    const auto divisor = std::int64_t{1} << shift;
    const auto truncated =
        (value / divisor) - static_cast<std::int64_t>(value < 0 && value % divisor != 0);
    const auto result = std::clamp(truncated, minimum, maximum);
    clipped += static_cast<unsigned>(result != truncated);
    return result;
  }
};

// Lane recurrence, not an instruction-by-instruction interpreter. DSP56300FM
// fractional products and full-accumulator data transfers: floor, then limit.
inline Prediction
predict(const Input &input, const Samples &source, const Coefficients &coefficients) {
  Prediction result;
  Transfer store{.scale = input.scale};
  constexpr std::int64_t gain_factor = 512;
  const auto gain = gain_factor * input.volume * input.volume;
  result.current = store.apply(2 * gain * coefficients.current);
  result.previous = store.apply(2 * gain * coefficients.previous);
  auto output_history = std::array{source.history[0], store.apply(word_unit * source.history[1])};
  auto input_history = std::array{source.history[2], source.history[3]};
  for (unsigned i = 0; i < result.output.size(); ++i) {
    const auto lane = i % 2;
    const auto accumulator = (2 * result.current * source.mixed.at(i)) -
                             (2 * result.previous * input_history.at(lane)) +
                             (2 * coefficients.feedback * output_history.at(lane));
    result.output.at(i) = store.apply(accumulator);
    input_history.at(lane) = source.mixed.at(i);
    output_history.at(lane) = result.output.at(i);
    if (lane == 1) { result.b = accumulator; }
  }
  result.history = {output_history[0], output_history[1], input_history[0], input_history[1]};
  result.a =
      (2 * result.current * source.mixed.back()) + (2 * coefficients.feedback * output_history[0]);
  result.clipped = store.clipped;
  return result;
}

inline unsigned raw(const std::int64_t value) {
  return static_cast<unsigned>(static_cast<std::uint64_t>(value) & word_mask);
}

inline std::uint64_t accumulator(const std::int64_t value) {
  return static_cast<std::uint64_t>(value) & accumulator_mask;
}
}
