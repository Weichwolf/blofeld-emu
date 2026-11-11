// SPDX-License-Identifier: GPL-3.0-only
#include "fixture.hpp"

extern "C" unsigned aot_first(const unsigned value, unsigned *state) noexcept {
  const auto mixed = aot_host(value, state);
  *state += value;
  return mixed ^ *state;
}
