// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "blofeld/mcu_flash_config.hpp"
#include "blofeld/mcu_memory.hpp"

#include <cstdint>

namespace blofeld {
void McuFlash::write_key_configuration(const std::uint8_t value) {
  if (active_ || buffer_state_ != BufferState::empty) {
    throw Fault("Unqualified MCU flash key access with a staged or executing command");
  }
  const bool before = (control_ & key_access) != 0;
  const bool requested = (value & key_access) != 0;
  if (before && !requested && !key_locked_ && key_words_ == 0) {
    throw Fault("Unqualified empty MCU flash key sequence closure");
  }
  if (!before && requested) { key_words_ = 0; }
  if (before && !requested) {
    constexpr unsigned complete_keys = 4;
    if (!key_locked_ && key_words_ == complete_keys) {
      config_.unsecure();
    } else {
      key_locked_ = true;
    }
  }
  constexpr unsigned writable = 0xe0;
  control_ = static_cast<std::uint8_t>(value & writable);
}

void McuFlash::write_key(const MemoryWrite word) {
  constexpr unsigned first = 0x7f00;
  constexpr unsigned key_bytes = 8;
  constexpr unsigned key_words = key_bytes / 2;
  constexpr unsigned word_mask = 0xffff;
  if (!config_.backdoor_enabled()) {
    throw Fault("Unsupported MCU flash key transaction with KEYEN disabled");
  }
  if (word.size != 2 || (word.address & 1U) != 0 || word.address < first ||
      word.address >= first + key_bytes) {
    throw Fault("Unqualified MCU flash key operand outside aligned key words");
  }
  if (key_locked_) { return; } // The specified security lock persists until reset.
  if (key_words_ == key_words || word.address != first + key_words_ * 2) {
    key_locked_ = true;
    return;
  }
  const auto high = memory_.read_flash(word.address);
  const auto low = memory_.read_flash(static_cast<std::uint16_t>(word.address + 1));
  const auto expected = (static_cast<unsigned>(high) << 8U) | low;
  const auto supplied = word.value & word_mask;
  if (supplied == 0 || supplied == word_mask || supplied != expected) {
    key_locked_ = true;
    return;
  }
  ++key_words_;
}
}
