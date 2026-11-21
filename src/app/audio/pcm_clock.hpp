// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/render_position.hpp"
#include "io/pcm_output.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::app {
class PcmClock {
public:
  struct Point {
    std::uint64_t epoch = 0, audio_epoch = 0, ticks = 0, host_ns = 0, uncertainty_ns = 0;
    bool operator==(const Point &) const = default;
  };

  explicit PcmClock(std::size_t history_frames);
  // One PCM owner appends and resets; readers may correlate native intervals concurrently.
  void append(vi::PcmOutput::Block block, std::uint64_t audio_epoch);
  void clear(std::uint64_t source_origin = 0) noexcept;
  [[nodiscard]] std::optional<Point> read(host::audio::RenderPosition position) const noexcept;

private:
  enum class Phase : std::uint8_t { empty, buffering, failed };
  [[noreturn]] void fail(const char *message);
  std::vector<std::atomic<std::uint64_t>> ticks_;
  std::atomic<std::uint64_t> source_version_{0}, epoch_{0}, audio_epoch_{0}, origin_{0},
      next_frame_{0};
  std::atomic<Phase> phase_{Phase::empty};
};
}
