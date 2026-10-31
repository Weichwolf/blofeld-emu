// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
// Exact ceiling of Hz * interval / UnitsPerSecond, including products wider
// than the counter. A uint32 interval bounds the remainder product to uint64.
template <std::uint32_t UnitsPerSecond>
[[nodiscard]] constexpr std::optional<std::uint64_t>
nand_interval_ticks(const std::uint64_t timebase_hz, const std::uint32_t interval) {
  static_assert(UnitsPerSecond != 0);
  const auto whole = timebase_hz / UnitsPerSecond;
  const auto remainder = (timebase_hz % UnitsPerSecond) * interval;
  const auto tail = (remainder / UnitsPerSecond) + (remainder % UnitsPerSecond != 0 ? 1U : 0U);
  if (interval != 0 && whole > (std::numeric_limits<std::uint64_t>::max() - tail) / interval) {
    return std::nullopt;
  }
  return (whole * interval) + tail;
}
}
