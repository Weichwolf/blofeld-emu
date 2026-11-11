// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>

struct Snapshot {
  std::array<unsigned, 8> values;
};

Snapshot aot_snapshot(const unsigned *state) noexcept;

extern "C" {
unsigned aot_host(unsigned value, unsigned *state) noexcept;
unsigned aot_first(unsigned value, unsigned *state) noexcept;
unsigned aot_second(unsigned value, unsigned *state) noexcept;
unsigned aot_initializations() noexcept;
void aot_test_event(unsigned event) noexcept;
}
