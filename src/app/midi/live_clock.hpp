// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld::app {
class LiveClock {
public:
  struct Grant {
    unsigned rate = 0;
    std::size_t queue_frames = 0;
    std::uint64_t uncertainty_ns = 0;
  };

  static constexpr std::uint64_t maximum_gap_ns = 10000000;
  explicit LiveClock(Grant grant);

  [[nodiscard]] std::uint64_t prime_ticks() const noexcept { return prime_; }

  [[nodiscard]] std::uint64_t uncertainty_ns() const noexcept { return uncertainty_; }

  [[nodiscard]] bool has_observation() const noexcept { return size_ != 0; }

  void observe(PcmClock::Point next);
  [[nodiscard]] std::optional<std::uint64_t> correlate(std::uint64_t host_ns,
                                                       std::uint64_t audio_epoch) const;

private:
  [[nodiscard]] const PcmClock::Point &point(std::size_t index) const noexcept;
  static constexpr std::size_t capacity = 64;
  std::array<PcmClock::Point, capacity> points_{};
  std::size_t head_ = 0, size_ = 0;
  std::uint64_t prime_ = 0;
  const std::uint64_t uncertainty_;
};
}
