// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_filter_model.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace blofeld::test::filters {
namespace {
// Recovered fractional-word gains and damping/cap slopes; these are oracle inputs.
constexpr std::int64_t quarter = half / 2;
constexpr std::int64_t interpolation_scale = 64;
constexpr std::int64_t interpolant_ceiling = 0x700000;
constexpr std::int64_t damping_bias = 0x3e0000;
constexpr std::int64_t resonance_slope = 0x420000;
constexpr std::int64_t cap_bias = 0x30000;
constexpr std::int64_t ordinary_cap_slope = 0x60000;
constexpr std::int64_t band_cap_slope = 0x7f0000;
constexpr std::int64_t high_cap_slope = 0x03d70a;
constexpr std::int64_t output_divisor = std::int64_t{1} << 22;

std::int64_t floor_div(const std::int64_t value, const std::int64_t divisor) {
  return value >= 0 ? value / divisor : -((-value + divisor - 1) / divisor);
}

std::int64_t limit(const std::int64_t value) {
  return std::clamp(value, -half, half - 1);
}

std::int64_t store(const std::int64_t accumulator) {
  return limit(floor_div(accumulator, word_unit));
}

Coefficients coefficients(const Input &input, const std::span<const unsigned> table) {
  const auto segment = floor_div(input.cutoff, segment_width);
  // Interpolate adjacent signed table words with the original fixed-point precision.
  const auto fraction = (input.cutoff - (segment * segment_width)) * interpolation_scale;
  const auto index = static_cast<std::size_t>(curve_base - table_base + segment);
  if (index + 1 >= table.size()) { throw Fault("Filter oracle lookup exceeds its pinned table"); }
  const auto left = signed_word(table[index]);
  const auto right = signed_word(table[index + 1]);
  const auto interpolated = (left * word_unit) + (2 * fraction * (right - left));
  const bool clips = input.kind == Kind::high || (input.cascade && input.kind != Kind::low);
  const auto bounded =
      clips ? std::min(interpolated, interpolant_ceiling * word_unit) : interpolated;
  const auto g = store(bounded);
  auto damping = (damping_bias * word_unit) - (2 * resonance_slope * input.resonance);
  const bool halved = damping < 0;
  if (halved) { damping = floor_div(damping, 2); }
  if (!input.cascade) { damping = store(damping) * word_unit; }
  auto cap = (-cap_bias * word_unit) + (2 * ordinary_cap_slope * input.cutoff);
  if (input.cascade && input.kind == Kind::band) {
    cap = (-half * word_unit) + (input.cutoff * word_unit) + (2 * band_cap_slope * input.cutoff);
  }
  if (input.cascade && input.kind == Kind::high) {
    cap = (-cap_bias * word_unit) + (2 * high_cap_slope * input.cutoff);
  }
  const bool capped = cap >= damping;
  damping = std::max(damping, cap);
  const auto d = store(damping);
  const auto e = store(damping - (2 * d * g));
  const auto feedback = (quarter * word_unit) - (2 * g * g) - (2 * e * g);
  // The wide core doubles the feedback contribution and rounds the output.
  const bool wide = feedback > quarter * word_unit;
  Coefficients result{.a1 = store((wide ? 1 : 2) * feedback),
                      .a2 = store((-quarter * word_unit) + (4 * e * g)),
                      .g = g,
                      .interpolant = bounded,
                      .wide = wide,
                      .clipped = interpolated != bounded,
                      .damping_halved = halved,
                      .cap_selected = capped};
  switch (input.kind) {
    case Kind::low:
      result.b0 = result.b2 = store(g * g);
      result.b1 = store(2 * g * g);
      break;
    case Kind::band:
      result.b0 = g;
      result.b2 = -g;
      break;
    case Kind::high:
      result.b0 = result.b2 = store((quarter * word_unit) - (g * g));
      result.b1 = store((-half * word_unit) + (2 * g * g));
      break;
    case Kind::notch:
      result.b0 = result.b2 = -quarter;
      result.b1 = store((half * word_unit) - (4 * g * g));
      break;
    default: throw Fault("Invalid independent filter kind");
  }
  return result;
}

Coefficients first_stage(Coefficients result) {
  result.a1 = store((half * word_unit) - (2 * result.interpolant));
  result.a2 = store((2 * result.interpolant) - (4 * result.g * result.g) - (quarter * word_unit));
  result.wide = false;
  return result;
}

Block filter(const Block &input,
             const Coefficients &coefficients,
             History &history,
             const bool twos) {
  Block output{};
  for (unsigned index = 0; index < block_length; ++index) {
    const auto sum = (coefficients.b0 * signed_word(input[index])) +
                     (coefficients.b1 * signed_word(history[1])) +
                     (coefficients.b2 * signed_word(history[0])) +
                     ((coefficients.wide ? 2 : 1) * coefficients.a1 * signed_word(history[3])) +
                     (coefficients.a2 * signed_word(history[2]));
    auto result = floor_div(sum, output_divisor);
    const auto remainder = sum - (result * output_divisor);
    if (coefficients.wide && (remainder > output_divisor / 2 ||
                              (remainder == output_divisor / 2 && (twos || result % 2 != 0)))) {
      ++result;
    }
    output[index] = raw(limit(result));
    history = {history[1], input[index], history[3], output[index]};
  }
  output[block_length] = input[block_length];
  return output;
}

Block samples(const unsigned pattern) {
  constexpr std::int64_t ramp_stride = 0x76543;
  constexpr std::int64_t ramp_bias = 0x654321;
  Block result{};
  for (unsigned index = 0; index <= block_length; ++index) {
    switch (pattern) {
      case 0: result[index] = index == 0 ? static_cast<unsigned>(quarter) : 0; break;
      case 1: result[index] = static_cast<unsigned>(index % 2 == 0 ? half - 1 : half); break;
      case 2: result[index] = raw(static_cast<std::int64_t>(index) - (block_length / 2)); break;
      case 3:
        result[index] = raw((static_cast<std::int64_t>(index) * ramp_stride) - ramp_bias);
        break;
      default: throw Fault("Invalid independent filter sample pattern");
    }
  }
  return result;
}
}

std::int64_t signed_word(const unsigned value) {
  return value < half ? value : static_cast<std::int64_t>(value) - word_unit;
}

unsigned raw(const std::int64_t value) {
  return static_cast<unsigned>(static_cast<std::uint64_t>(value) & word_mask);
}

Prediction predict(const Input &input, const std::span<const unsigned> table) {
  Prediction result;
  result.final = coefficients(input, table);
  result.first = input.cascade ? first_stage(result.final) : result.final;
  result.input = samples(input.pattern);
  result.intermediate = filter(result.input, result.first, result.first_history, input.twos);
  result.output = input.cascade
                      ? filter(result.intermediate, result.final, result.second_history, input.twos)
                      : result.intermediate;
  result.context = " kind=" + std::to_string(static_cast<unsigned>(input.kind)) +
                   " f=" + std::to_string(input.cutoff) + " r=" + std::to_string(input.resonance) +
                   " RM=" + std::to_string(static_cast<int>(input.twos)) +
                   (input.cascade ? " cascade" : " single");
  return result;
}
}
