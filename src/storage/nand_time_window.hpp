// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>

namespace blofeld {
// Inclusive bounds on an event time, not an observed edge. Equal endpoints
// describe exact observations; [0,end] can conservatively cover opaque CE history.
struct NandTimeWindow {
  std::uint64_t earliest = 0;
  std::uint64_t latest = 0;
  bool operator==(const NandTimeWindow &) const = default;
};
}
