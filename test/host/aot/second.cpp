// SPDX-License-Identifier: GPL-3.0-only
#include "fixture.hpp"

Snapshot aot_snapshot(const unsigned *state) noexcept {
  Snapshot result{};
  for (unsigned index = 0; index < 8; ++index) { result.values[index] = *state + index; }
  return result;
}

extern "C" unsigned aot_second(const unsigned value, unsigned *state) noexcept {
  const auto first = aot_first(value, state);
  return first + aot_host(first, state);
}
