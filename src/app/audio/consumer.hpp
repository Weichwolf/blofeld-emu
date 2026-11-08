// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/playback.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::app {
struct PcmStatistics {
  std::uint64_t accepted_frames = 0, device_underruns = 0;
  std::optional<double> production_reserve;
};

// Composition only: preserve VI sequence/time, feed the generic host consumer,
// and measure host work between submissions without touching either Core.
class AudioConsumer {
public:
  explicit AudioConsumer(host::audio::Playback &playback) : playback_(playback) {}

  [[nodiscard]] vi::PcmOutput output() noexcept;
  [[nodiscard]] PcmStatistics statistics() const noexcept;

private:
  [[nodiscard]] bool write(vi::PcmOutput::Block block, const std::stop_token &stop);
  host::audio::Playback &playback_;
  std::optional<std::uint64_t> epoch_;
  std::optional<std::chrono::steady_clock::time_point> previous_;
  std::uint64_t next_frame_ = 0, active_ns_ = 0, timed_frames_ = 0;
  std::atomic<std::uint64_t> accepted_{0}, underruns_{0}, measured_ns_{0}, measured_frames_{0};
};
}
