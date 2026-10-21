// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/pcm_recording.hpp"
#include "blofeld/pcm_wave.hpp"
#include "blofeld/private_file.hpp"

#include <filesystem>

namespace blofeld {
void save_pcm_wave(const std::filesystem::path &path,
                   const PcmRecording &recording,
                   const PcmWaveClock clock,
                   const PrivateFileObserver &observer) {
  if (path.extension() != ".wav") { throw Fault("WAV output requires the .wav extension"); }
  const auto bytes = encode_pcm_wave(recording, clock);
  write_private_file(path, bytes, observer);
}
}
