// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/shi.hpp"

#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::vi::mcu {
// Port B firmware adapter: B1 selects, B2 clocks, B4 carries serial data.
// Collect locally; publish completed words to SHI without per-bit shared work.
class BootPort {
public:
  struct Write {
    std::uint8_t direction = 0, known = 0, value = 0;

    [[nodiscard]] constexpr bool selected() const noexcept {
      constexpr std::uint8_t transaction_mask = 0x16;
      constexpr std::uint8_t select_bit = 2;
      return (direction & known & transaction_mask) == transaction_mask &&
             (value & select_bit) == 0;
    }
  };

  struct Snapshot {
    std::uint8_t previous = 0;
    bool selected = false, pending_word = false;
    std::optional<Shi::Mode> mode;
    unsigned bits = 0;
    std::uint32_t word = 0;
  };

  [[nodiscard]] Snapshot save() const noexcept { return state_; }

  void restore(const Snapshot &snapshot) noexcept { state_ = snapshot; }

  explicit BootPort(Shi &shi) : shi_(shi) {}

  [[nodiscard]] bool write(Write access) noexcept;
  void reset() noexcept;

  [[nodiscard]] bool pending() const noexcept {
    return state_.pending_word || (state_.selected && !state_.mode);
  }

  [[nodiscard]] bool complete(const std::stop_token &stop);

private:
  Shi &shi_;
  Snapshot state_;
};
}
