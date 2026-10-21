// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/sample_wave.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::samples {
namespace {
// YIN difference/cumulative-mean threshold with raw-difference interpolation.
// No global-minimum fallback: uncertain/nonperiodic material needs a manual root.
constexpr double threshold = 0.1;
constexpr unsigned analysis_rate_limit = 12000;
constexpr std::size_t maximum_analysis = 16384;
constexpr std::size_t maximum_window = 8192;
constexpr unsigned minimum_frequency = 8;

std::vector<double> analysis(const SampleWave &wave, const unsigned stride) {
  const auto count = std::min(maximum_analysis, wave.mono.size() / stride);
  std::vector<double> result(count);
  const auto peak = mono_peak(wave);
  if (peak == 0) { return {}; }
  for (std::size_t i = 0; i < count; ++i) {
    for (unsigned j = 0; j < stride; ++j) {
      result[i] += (wave.mono[(i * stride) + j] / peak) / static_cast<double>(stride);
    }
  }
  return result;
}

std::vector<double> differences(const std::span<const double> data, const std::size_t last_lag) {
  const auto window = std::min(maximum_window, data.size() - last_lag);
  std::vector<double> result(last_lag + 1);
  for (std::size_t lag = 1; lag <= last_lag; ++lag) {
    for (std::size_t i = 0; i < window; ++i) {
      const auto delta = data[i] - data[i + lag];
      result[lag] += delta * delta;
    }
  }
  return result;
}

double interpolated(const std::span<const double> raw, const std::size_t lag) {
  const double bend = raw[lag - 1] - (2 * raw[lag]) + raw[lag + 1];
  if (bend <= 0) { return static_cast<double>(lag); }
  const auto adjustment = std::clamp((raw[lag - 1] - raw[lag + 1]) / (2 * bend), -0.5, 0.5);
  return static_cast<double>(lag) + adjustment;
}
}

std::optional<SamplePitch> detect_sample_pitch(const SampleWave &wave) {
  if (wave.sample_rate == 0) { return std::nullopt; }
  const auto stride = 1 + ((wave.sample_rate - 1) / analysis_rate_limit);
  const auto data = analysis(wave, stride);
  if (data.size() < 16) { return std::nullopt; }
  const auto rate = static_cast<double>(wave.sample_rate) / stride;
  const auto last_lag =
      std::min(data.size() / 2, static_cast<std::size_t>(rate / minimum_frequency));
  if (last_lag < 4) { return std::nullopt; }
  const auto raw = differences(data, last_lag);
  std::vector<double> normalized(raw.size(), 1.0);
  double cumulative = 0;
  for (std::size_t lag = 1; lag <= last_lag; ++lag) {
    cumulative += raw[lag];
    if (cumulative != 0) { normalized[lag] = raw[lag] * static_cast<double>(lag) / cumulative; }
  }
  for (std::size_t lag = 2; lag < last_lag; ++lag) {
    if (normalized[lag] >= threshold) { continue; }
    while (lag + 1 < last_lag && normalized[lag + 1] < normalized[lag]) { ++lag; }
    const auto hz = rate / interpolated(raw, lag);
    constexpr double concert_a_hz = 440;
    constexpr double concert_a_key = 69;
    const auto note = concert_a_key + (12 * std::log2(hz / concert_a_hz));
    return SamplePitch{.midi_note = note, .frequency_hz = hz, .periodicity = 1 - normalized[lag]};
  }
  return std::nullopt;
}
}
