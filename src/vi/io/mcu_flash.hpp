// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/flash_image.hpp"

#include <cstdint>

namespace blofeld::vi {
// UF32 FTS32K normal-mode register bank. One MCU execution owner.
// Rejected array writes set the documented error flags without changing data.
// Clocked program/erase, security-key access and IRQ delivery are not yet bound;
// attempts to enter those paths fail instead of reporting command completion.
class McuFlash {
public:
  struct Write {
    std::uint16_t offset = 0;
    std::uint16_t value = 0;
    std::uint8_t bytes = 1;
  };

  explicit McuFlash(const FlashImage &image) noexcept;
  void reset(const FlashImage &image) noexcept;

  [[nodiscard]] static constexpr bool contains(const std::uint16_t offset) noexcept {
    return offset >= register_first && offset <= register_last;
  }

  [[nodiscard]] bool read(std::uint16_t offset, std::uint8_t &value) const noexcept;
  bool write(Write access) noexcept;
  // Offset is physical in the 32 KiB image, including writes through aliases.
  bool write_array(Write access) noexcept;

private:
  static constexpr std::uint16_t register_first = 0x100, register_last = 0x10f;
  static constexpr std::uint8_t ready = 0xc0;
  bool protect(std::uint8_t value) noexcept;
  [[nodiscard]] bool protected_at(std::uint16_t offset) const noexcept;
  std::uint8_t divider_ = 0;
  std::uint8_t security_ = 0;
  std::uint8_t protection_ = 0;
  std::uint8_t status_ = ready;
  std::uint8_t command_ = 0;
  bool security_known_ = false;
  bool protection_known_ = false;
};
}
