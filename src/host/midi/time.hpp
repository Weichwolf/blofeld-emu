// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld::host::midi {
struct Time {
  std::uint64_t microseconds = 0;
  std::uint32_t fraction = 0, divisor = 1;
  bool operator==(const Time &) const = default;
  [[nodiscard]] std::uint64_t ceil_ticks(std::uint32_t frequency) const;
};
}
