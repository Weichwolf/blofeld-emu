// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_bdc_wire.hpp"

namespace blofeld {
void Hcs08BdcWire::sync_clock() {
  constexpr unsigned response_delay = 16;
  constexpr unsigned response_low = 128;
  constexpr unsigned response_pulse = 1;
  if (state_.sync == Hcs08BdcSync::wait_release) { return; }
  if (!state_.sync_release) { state_.sync_release = state_.cycles; }
  const auto age = state_.cycles - *state_.sync_release;
  if (age == response_delay) {
    state_.target = Hcs08BdcDrive::low;
    state_.sync = Hcs08BdcSync::response;
  } else if (age == response_delay + response_low) {
    state_.target = Hcs08BdcDrive::high;
  } else if (age == response_delay + response_low + response_pulse) {
    state_.target = Hcs08BdcDrive::released;
    state_.sync = Hcs08BdcSync::idle;
    state_.sync_release.reset();
    state_.last_fall.reset();
    ++state_.syncs;
  }
}
} // namespace blofeld
