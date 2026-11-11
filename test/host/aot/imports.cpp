// SPDX-License-Identifier: GPL-3.0-only
#include <cstdint>

extern "C" {
unsigned aot_native_step(unsigned value, unsigned *state) noexcept;
unsigned aot_native_send(void *channel, unsigned value, std::uint64_t ticks) noexcept;
void aot_native_event(unsigned event) noexcept;
}

namespace {
[[gnu::constructor]] void begin() {
  aot_native_event(1);
}

[[gnu::destructor]] void end() {
  aot_native_event(2);
}
}

extern "C" unsigned aot_import_step(const unsigned value, unsigned *state) noexcept {
  return aot_native_step(aot_native_step(value, state), state);
}

extern "C" unsigned
aot_import_send(void *channel, const unsigned value, const std::uint64_t ticks) noexcept {
  return aot_native_send(channel, value, ticks);
}
