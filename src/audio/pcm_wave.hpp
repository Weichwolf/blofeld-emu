// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_recording.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

namespace blofeld {
struct PcmWaveClock {
  std::uint64_t timebase_hz = 0;
  std::uint32_t sample_rate = 0;
};

// WAVEFORMATEXTENSIBLE signed integer PCM, 32-bit containers, original precision.
// No speaker positions, inferred rate, resampling or incomplete-frame fill.
[[nodiscard]] std::vector<std::uint8_t> encode_pcm_wave(const PcmRecording &recording,
                                                        PcmWaveClock clock);
void save_pcm_wave(const std::filesystem::path &path,
                   const PcmRecording &recording,
                   PcmWaveClock clock,
                   const PrivateFileObserver &observer = {});
}
