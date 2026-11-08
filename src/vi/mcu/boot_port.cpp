// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/boot_port.hpp"

#include <cstdint>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint8_t select_bit = 2, clock_bit = 4, data_bit = 0x10;
constexpr std::uint8_t transaction_mask = select_bit | clock_bit | data_bit;
}

bool BootPort::write(const Write access) noexcept {
  const auto previous = previous_;
  previous_ = access.value;
  const bool selected = (access.direction & access.known & transaction_mask) == transaction_mask &&
                        (access.value & select_bit) == 0;
  if (selected != selected_) {
    mode_ = shi_.select(selected);
    selected_ = selected;
    bits_ = 0;
    word_ = 0;
    return true;
  }
  if (!mode_ || ((access.value ^ previous) & clock_bit) == 0 ||
      ((access.value & clock_bit) == 0) != mode_->falling) {
    return true;
  }
  // CPHA=0 requires a selection boundary between words (DSP56371UM 7.4.5.1).
  if (bits_ == mode_->bits) { return false; }
  word_ = (word_ << 1U) | ((access.value & data_bit) != 0 ? 1U : 0U);
  ++bits_;
  if (bits_ != mode_->bits) { return true; }
  const auto accepted = shi_.receive(word_);
  if (mode_->continuous) {
    bits_ = 0;
    word_ = 0;
  }
  return accepted;
}
}
