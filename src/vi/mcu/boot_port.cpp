// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/boot_port.hpp"

#include "io/shi.hpp"
#include "mcu/channels.hpp"

#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint8_t clock_bit = 4, data_bit = 0x10;
}

void BootPort::reset() noexcept {
  state_.previous = 0;
  state_.selected = state_.pending_word = false;
  state_.mode.reset();
  state_.bits = 0;
  state_.word = 0;
}

bool BootPort::complete(const std::stop_token &stop) {
  if (!pending()) { return true; }
  if (!state_.mode) {
    Shi::Mode mode;
    if (!blofeld_mcu_shi_mode(&shi_, mode, stop)) { return false; }
    state_.mode = mode;
  }
  if (!state_.pending_word) { return true; }
  if (!blofeld_mcu_shi_submit(&shi_, state_.word, stop)) { return false; }
  state_.pending_word = false;
  if (state_.mode->continuous) {
    state_.bits = 0;
    state_.word = 0;
  }
  return true;
}

bool BootPort::write(const Write access) noexcept {
  if (state_.pending_word) { return false; }
  const auto previous = state_.previous;
  state_.previous = access.value;
  const bool selected = access.selected();
  if (selected != state_.selected) {
    Shi::Mode mode;
    state_.mode =
        blofeld_mcu_shi_select(&shi_, selected, mode) ? std::optional{mode} : std::nullopt;
    state_.selected = selected;
    state_.bits = 0;
    state_.word = 0;
    return true;
  }
  if (!state_.mode || ((access.value ^ previous) & clock_bit) == 0 ||
      ((access.value & clock_bit) == 0) != state_.mode->falling) {
    return true;
  }
  // CPHA=0 requires a selection boundary between words (DSP56371UM 7.4.5.1).
  if (state_.bits == state_.mode->bits) { return false; }
  state_.word = (state_.word << 1U) | ((access.value & data_bit) != 0 ? 1U : 0U);
  ++state_.bits;
  if (state_.bits != state_.mode->bits) { return true; }
  state_.pending_word = true;
  return true;
}
}
