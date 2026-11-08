// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/boot_port.hpp"

#include <cstdint>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t data_address = 1, direction_address = 3;
constexpr std::uint8_t select_bit = 2, clock_bit = 4, data_bit = 0x10;
constexpr std::uint8_t transaction_mask = select_bit | clock_bit | data_bit;
constexpr std::uint8_t all_bits = 0xff;
}

bool BootPort::read(const std::uint16_t offset, std::uint8_t &value) const noexcept {
  if (offset == direction_address) {
    value = direction_;
    return true;
  }
  if (offset != data_address || !latch_known_ || direction_ != all_bits) { return false; }
  value = latch_;
  return true;
}

bool BootPort::write(const Write access) noexcept {
  const auto previous = latch_;
  if (access.offset == direction_address) {
    direction_ = access.value;
  } else if (access.offset == data_address) {
    latch_ = access.value;
    latch_known_ = true;
  } else {
    return false;
  }
  return update(previous);
}

bool BootPort::update(const std::uint8_t previous) noexcept {
  const bool selected = latch_known_ && (direction_ & transaction_mask) == transaction_mask &&
                        (latch_ & select_bit) == 0;
  if (selected != selected_) {
    mode_ = shi_.select(selected);
    selected_ = selected;
    bits_ = 0;
    word_ = 0;
    return true;
  }
  if (!mode_ || ((latch_ ^ previous) & clock_bit) == 0 ||
      ((latch_ & clock_bit) == 0) != mode_->falling) {
    return true;
  }
  // CPHA=0 requires a selection boundary between words (DSP56371UM 7.4.5.1).
  if (bits_ == mode_->bits) { return false; }
  word_ = (word_ << 1U) | ((latch_ & data_bit) != 0 ? 1U : 0U);
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
