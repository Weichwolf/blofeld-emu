// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::samples {
// Offline editor resource limits, independent of the Blofeld sample budget.
inline constexpr std::size_t maximum_wave_bytes = std::size_t{256} * 1024 * 1024;
inline constexpr std::size_t maximum_wave_frames = std::size_t{32} * 1024 * 1024;

struct WaveLoop {
  std::uint32_t identifier = 0;
  std::uint32_t type = 0;
  std::uint32_t start = 0;
  std::uint32_t end_inclusive = 0;
  std::uint32_t fraction = 0;
  std::uint32_t play_count = 0;
};

struct WaveInstrument {
  unsigned note = 0;
  int correction_cents = 0;
  int gain_db = 0;
  unsigned lowest_key = 0;
  unsigned highest_key = 0;
  unsigned lowest_velocity = 0;
  unsigned highest_velocity = 0;
};

struct SampleWave {
  unsigned sample_rate = 0;
  unsigned source_channels = 0;
  unsigned container_bits = 0;
  unsigned valid_bits = 0;
  std::uint32_t channel_mask = 0;
  bool floating_point = false;
  // Channel arithmetic mean, without clipping, gain, resampling or normalization.
  std::vector<double> mono;
  std::optional<double> sampler_root;
  std::optional<WaveInstrument> instrument;
  std::vector<WaveLoop> loops;
};

struct SamplePitch {
  double midi_note = 0;
  double frequency_hz = 0;
  double periodicity = 0;
};

[[nodiscard]] SampleWave decode_sample_wave(std::span<const std::uint8_t> bytes);
[[nodiscard]] SampleWave load_sample_wave(const std::filesystem::path &path);
[[nodiscard]] double mono_peak(const SampleWave &wave);
[[nodiscard]] std::optional<double> metadata_root(const SampleWave &wave);
// Bounded offline periodicity estimate, not the original Spectre pitch algorithm.
[[nodiscard]] std::optional<SamplePitch> detect_sample_pitch(const SampleWave &wave);
}
