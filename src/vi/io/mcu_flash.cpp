// SPDX-License-Identifier: GPL-3.0-only
#include "io/mcu_flash.hpp"

#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace blofeld::vi {
namespace {
constexpr std::uint16_t clock_divider = 0x100, security = 0x101, configuration = 0x103;
constexpr std::uint16_t protection = 0x104, status = 0x105, command = 0x106;
constexpr std::size_t protection_byte = 0x7f0d, security_byte = 0x7f0f;
constexpr std::uint8_t all_bits = 0xff, configuration_mask = 0xe0, mass_erase = 0x41;
constexpr std::uint8_t loaded = 0x80, access_error = 0x10, violation = 0x20;
constexpr std::uint8_t open = 0x80, high_disabled = 0x20, low_disabled = 4;
constexpr std::uint8_t high_size_mask = 0x18, low_size_mask = 3;

constexpr unsigned scenario(const std::uint8_t value) noexcept {
  return ((value & open) != 0 ? 4U : 0U) | ((value & high_disabled) != 0 ? 2U : 0U) |
         ((value & low_disabled) != 0 ? 1U : 0U);
}
}

McuFlash::McuFlash(const FlashImage &image) noexcept {
  reset(image);
}

void McuFlash::reset(const FlashImage &image) noexcept {
  divider_ = 0;
  security_ = image.bytes[security_byte];
  protection_ = image.bytes[protection_byte];
  security_known_ = image.known[security_byte] != 0;
  protection_known_ = image.known[protection_byte] != 0;
  status_ = ready;
  command_ = 0;
}

bool McuFlash::read(const std::uint16_t offset, std::uint8_t &value) const noexcept {
  if (!contains(offset)) { return false; }
  switch (offset) {
    case clock_divider: value = divider_; return true;
    case security: value = security_; return security_known_;
    case protection: value = protection_; return protection_known_;
    case status: value = status_; return true;
    case command: value = command_; return true;
    // Reserved and normal-mode FADDR/FDATA registers read zero.
    default: value = 0; return true;
  }
}

bool McuFlash::protect(const std::uint8_t value) noexcept {
  if (!protection_known_) { return false; }
  // NXP FTS32K Table 3-9: allowed destination scenarios for each source.
  constexpr std::array<std::uint8_t, 8> transitions{0x0f, 0x0a, 0x0c, 0x08, 0x18, 0x3c, 0x5a, 0xff};
  if ((transitions[scenario(protection_)] & (1U << scenario(value))) == 0) { return true; }
  const auto locked =
      static_cast<std::uint8_t>(((protection_ & high_disabled) == 0 ? high_size_mask : 0) |
                                ((protection_ & low_disabled) == 0 ? low_size_mask : 0));
  protection_ = static_cast<std::uint8_t>((value & ~locked) | (protection_ & locked));
  return true;
}

bool McuFlash::protected_at(const std::uint16_t offset) const noexcept {
  const auto high_bytes = 2048U << ((protection_ & high_size_mask) >> 3U);
  const auto low_bytes = 512U << (protection_ & low_size_mask);
  const auto in_range =
      ((protection_ & high_disabled) == 0 && offset >= FlashImage::byte_count - high_bytes) ||
      ((protection_ & low_disabled) == 0 && offset < low_bytes);
  return (protection_ & open) != 0 ? in_range : !in_range;
}

bool McuFlash::write_array(const Write access) noexcept {
  if (access.offset >= FlashImage::byte_count || (access.bytes != 1 && access.bytes != 2)) {
    return false;
  }
  if (access.bytes == 1 || (access.offset & 1) != 0 || (divider_ & loaded) == 0) {
    status_ |= access_error;
    return true;
  }
  if (!protection_known_) { return false; }
  if (protected_at(access.offset)) {
    status_ |= violation;
    return true;
  }
  return false; // A valid word starts a clocked command, not an immediate RAM store.
}

bool McuFlash::write(const Write access) noexcept {
  if (!contains(access.offset) || access.bytes != 1 || access.value > all_bits) { return false; }
  const auto value = static_cast<std::uint8_t>(access.value);
  switch (access.offset) {
    case clock_divider:
      if ((divider_ & loaded) == 0) { divider_ = value | loaded; }
      return true;
    case security: return true; // Read-only in normal mode.
    case configuration: return (value & configuration_mask) == 0;
    case protection: return protect(value);
    case status:
      if ((value & loaded) != 0) { return false; }
      status_ &= static_cast<std::uint8_t>(~(value & (access_error | violation)));
      return true;
    case command: {
      constexpr std::uint8_t mask = 0x65;
      const auto selected = static_cast<std::uint8_t>(value & mask);
      if (selected == 5 || selected == 0x20 || selected == 0x40 || selected == mass_erase) {
        return false;
      }
      command_ = selected;
      status_ |= access_error;
      return true;
    }
    default: return true; // Reserved and normal-mode FADDR/FDATA writes are ignored.
  }
}
}
