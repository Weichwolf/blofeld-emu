// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_clock.hpp"
#include "audio/playback.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
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
  explicit AudioConsumer(host::audio::Playback &playback, bool deferred = false);

  // Publish the original-time boundary before releasing the MIDI admission frontier.
  void begin_at(std::uint64_t ticks);
  [[nodiscard]] bool has_pcm() const noexcept;

  [[nodiscard]] vi::PcmOutput output() noexcept;
  [[nodiscard]] PcmStatistics statistics() const noexcept;
  [[nodiscard]] std::optional<PcmClock::Point> clock() const noexcept;

private:
  enum class Phase : std::uint8_t { waiting, writing, closed, failed };
  [[nodiscard]] bool write(vi::PcmOutput::Block block, const std::stop_token &stop);
  [[nodiscard]] bool submit(vi::PcmOutput::Block block, const std::stop_token &stop);
  [[nodiscard]] bool source(vi::PcmOutput::Block block, const std::stop_token &stop);
  host::audio::Playback &playback_;
  PcmClock clock_;
  std::optional<host::audio::Playback::Producer> producer_;
  std::optional<std::uint64_t> epoch_, last_tick_;
  std::optional<std::chrono::steady_clock::time_point> previous_;
  std::uint64_t next_frame_ = 0, active_ns_ = 0, timed_frames_ = 0;
  std::uint64_t audio_epoch_ = 0;
  std::atomic<Phase> phase_;
  std::atomic<std::uint64_t> begin_{std::numeric_limits<std::uint64_t>::max()}, observed_{0};
  std::atomic<bool> pcm_{false};
  std::atomic<std::uint64_t> accepted_{0}, underruns_{0}, measured_ns_{0}, measured_frames_{0};
};
}
