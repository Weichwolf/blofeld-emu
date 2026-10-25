// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <cstdint>

namespace blofeld {
// Configuration register only. Its consuming watchdog/STOP/BDC owners remain
// separate; writing this register never synthesizes their behavior or clocks.
class Hcs08SystemOptions {
public:
  explicit Hcs08SystemOptions(const Hcs08FlashContract contract) {
    if (contract != Hcs08FlashContract::candidate_gt8a) {
      throw Fault("HCS08 SOPT requires the explicit candidate GT8A contract");
    }
  }

  void write(const ByteWrite access, const ExecutionContext context) {
    if (access.address != address) {
      throw Fault("HCS08 SOPT write address=" + hex(access.address) + " pc=" + hex(context.pc));
    }
    if (written_) { return; }
    constexpr unsigned reserved_one = 0x11;
    constexpr unsigned unimplemented = 0x0c;
    if ((access.value & reserved_one) != reserved_one || (access.value & unimplemented) != 0) {
      throw Fault("Unqualified HCS08 SOPT reserved-bit write address=" + hex(access.address) +
                  " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr));
    }
    value_ = access.value;
    written_ = true;
  }

  [[nodiscard]] std::uint8_t read() const { return value_; }

  [[nodiscard]] bool written() const { return written_; }

  void reset() {
    value_ = reset_value;
    written_ = false;
  }

  static constexpr unsigned address = 0x1802, reset_value = 0xd3;

private:
  std::uint8_t value_ = reset_value;
  bool written_ = false;
};
} // namespace blofeld
