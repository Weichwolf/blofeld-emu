// SPDX-License-Identifier: GPL-3.0-only
#include "midi/time.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace blofeld::host::midi {
std::uint64_t Time::ceil_ticks(const std::uint32_t frequency) const {
  if (frequency == 0 || divisor == 0 || fraction >= divisor) {
    throw std::invalid_argument("MIDI time or target clock is invalid");
  }
  constexpr std::uint64_t microseconds_per_second = 1000000;
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  const auto seconds = microseconds / microseconds_per_second;
  const auto submicro = std::uint64_t{fraction} * frequency;
  const auto partial =
      ((microseconds % microseconds_per_second) * frequency) + (submicro / divisor);
  auto tail = partial / microseconds_per_second;
  if (partial % microseconds_per_second != 0 || submicro % divisor != 0) { ++tail; }
  if (seconds > (maximum - tail) / frequency) {
    throw std::overflow_error("MIDI timestamp exceeds the target clock range");
  }
  return (seconds * frequency) + tail;
}
}
