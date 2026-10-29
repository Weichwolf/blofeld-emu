// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_square_clock.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <limits>
#include <numeric>
#include <string>

namespace blofeld::reference {
SquareClock::SquareClock(const SquareClockConfig config)
    : config_(config), current_{.ticks = config.epoch_ticks, .level = config.initial_level} {
  if (config.frequency_hz == 0 || config.timebase_hz == 0 ||
      config.timebase_hz % config.frequency_hz != 0) {
    throw Fault("Reference square clock requires a nonzero exact frequency/timebase");
  }
  const auto period = config.timebase_hz / config.frequency_hz;
  if (period % 2 != 0) {
    throw Fault("Reference square clock half-period is not representable in its timebase");
  }
  half_period_ = period / 2;
  static_cast<void>(next());
}

std::uint64_t SquareClock::extend_timebase(const std::uint64_t existing_hz,
                                           const std::uint64_t frequency_hz) {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (existing_hz == 0 || frequency_hz == 0 || frequency_hz > maximum / 2) {
    throw Fault("Reference square clock has an invalid timebase/frequency");
  }
  const auto edge_hz = frequency_hz * 2;
  const auto factor = edge_hz / std::gcd(existing_hz, edge_hz);
  if (factor == 0 || existing_hz > maximum / factor) {
    throw Fault("Reference square clock common timebase overflows");
  }
  return existing_hz * factor;
}

SquareClockLevel SquareClock::next() const {
  if (current_.ticks > std::numeric_limits<std::uint64_t>::max() - half_period_) {
    throw Fault("Reference square clock transition overflows after tick=" +
                std::to_string(current_.ticks));
  }
  return {.ticks = current_.ticks + half_period_, .level = !current_.level};
}

bool SquareClock::level_at(const std::uint64_t ticks) const {
  if (ticks < config_.epoch_ticks) {
    throw Fault("Reference square clock queried before its declared epoch");
  }
  return config_.initial_level != (((ticks - config_.epoch_ticks) / half_period_) % 2 != 0);
}

void SquareClock::consume(const std::uint64_t ticks) {
  const auto edge = next();
  if (ticks != edge.ticks) {
    throw Fault("Reference square clock transition mismatch expected=" +
                std::to_string(edge.ticks) + " actual=" + std::to_string(ticks));
  }
  current_ = edge;
}
}
