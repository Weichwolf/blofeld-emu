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
  struct Window {
    std::uint64_t host_ns = 0, ticks = 0;
    bool operator==(const Window &) const = default;
  };

  struct Grant {
    unsigned rate = 0;
    std::size_t queue_frames = 0;
    std::uint64_t uncertainty_ns = 0;
  };

  static constexpr std::uint64_t delivery_ns = 1000000, maximum_gap_ns = 10000000;
  explicit LiveClock(Grant grant);

  [[nodiscard]] std::uint64_t prime_ticks() const noexcept { return prime_; }

  [[nodiscard]] std::uint64_t lead_ticks() const noexcept { return lead_; }

  [[nodiscard]] std::uint64_t uncertainty_ns() const noexcept { return uncertainty_; }

  void observe(PcmClock::Point next);
  [[nodiscard]] std::optional<Window> window(std::uint64_t now);
  [[nodiscard]] std::uint64_t event(std::uint64_t timestamp);
  void commit(Window window);

private:
  [[nodiscard]] std::optional<std::uint64_t> at(std::uint64_t timestamp) const;
  [[nodiscard]] const PcmClock::Point &point(std::size_t index) const noexcept;
  static constexpr std::size_t capacity = 64;
  std::array<PcmClock::Point, capacity> points_{};
  std::size_t head_ = 0, size_ = 0;
  std::uint64_t prime_ = 0, lead_ = 0;
  const std::uint64_t uncertainty_;
  std::optional<std::uint64_t> committed_, previous_event_, previous_poll_;
};
}
