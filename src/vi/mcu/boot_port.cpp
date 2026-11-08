// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/boot_port.hpp"

#include <cstdint>
#include <stop_token>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint8_t select_bit = 2, clock_bit = 4, data_bit = 0x10;
constexpr std::uint8_t transaction_mask = select_bit | clock_bit | data_bit;
}

void BootPort::reset() noexcept {
  previous_ = 0;
  selected_ = pending_word_ = false;
  mode_.reset();
  bits_ = 0;
  word_ = 0;
}

bool BootPort::complete(const std::stop_token &stop) {
  if (!pending()) { return true; }
  if (!mode_) {
    mode_ = shi_.wait_mode(stop);
    if (!mode_) { return false; }
  }
  if (!pending_word_) { return true; }
  if (!shi_.submit(word_, stop).accepted) { return false; }
  pending_word_ = false;
  if (mode_->continuous) {
    bits_ = 0;
    word_ = 0;
  }
  return true;
}

bool BootPort::write(const Write access) noexcept {
  if (pending_word_) { return false; }
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
  pending_word_ = true;
  return true;
}
}
