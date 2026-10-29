// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld::reference {
struct SquareClockConfig {
  std::uint64_t timebase_hz = 0;
  std::uint64_t frequency_hz = 0;
  std::uint64_t epoch_ticks = 0;
  bool initial_level = false;
  bool operator==(const SquareClockConfig &) const = default;
};

struct SquareClockLevel {
  std::uint64_t ticks = 0;
  bool level = false;
  bool operator==(const SquareClockLevel &) const = default;
};

// Explicit digital reference source. It has no guest reset or host-clock input.
// The epoch establishes a level; the first transition is one half-period later.
class SquareClock {
public:
  explicit SquareClock(SquareClockConfig config);
  [[nodiscard]] static std::uint64_t extend_timebase(std::uint64_t existing_hz,
                                                     std::uint64_t frequency_hz);

  [[nodiscard]] SquareClockLevel current() const { return current_; }

  [[nodiscard]] SquareClockLevel next() const;
  [[nodiscard]] bool level_at(std::uint64_t ticks) const;
  void consume(std::uint64_t ticks);

  [[nodiscard]] const SquareClockConfig &config() const { return config_; }

  [[nodiscard]] std::uint64_t half_period_ticks() const { return half_period_; }

  bool operator==(const SquareClock &) const = default;

private:
  SquareClockConfig config_;
  SquareClockLevel current_;
  std::uint64_t half_period_ = 0;
};
}
