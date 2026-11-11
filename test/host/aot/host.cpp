// SPDX-License-Identifier: GPL-3.0-only
#include "fixture.hpp"

namespace {
unsigned initializations = 0;

[[gnu::constructor]] void begin() {
  ++initializations;
  aot_test_event(1);
}

[[gnu::destructor]] void end() {
  aot_test_event(2);
}
}

extern "C" unsigned aot_host(const unsigned value, unsigned *state) noexcept {
  *state = (*state * 3) ^ value;
  return aot_snapshot(state).values[7];
}

extern "C" unsigned aot_initializations() noexcept {
  return initializations;
}
