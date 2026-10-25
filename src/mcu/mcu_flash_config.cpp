// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_flash_config.hpp"

#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned loaded = 0x80;
constexpr unsigned divider_bits = 0x3f;
constexpr unsigned prescale = 0x40;
constexpr unsigned open = 0x80;
constexpr unsigned high_disabled = 0x20;
constexpr unsigned low_disabled = 4;
constexpr unsigned disabled = high_disabled | low_disabled;
constexpr unsigned high_size = 0x18;
constexpr unsigned low_size = 3;
constexpr unsigned flash_bytes = 0x8000;
constexpr unsigned low_sector_bytes = 512;
constexpr unsigned high_sector_bytes = 2048;
constexpr unsigned key_mask = 0xc0;

struct ProtectionTransition {
  unsigned before = 0;
  unsigned after = 0;
};

bool allowed_scenario(const ProtectionTransition change) {
  const auto previous_disabled = change.before & disabled;
  const auto next_disabled = change.after & disabled;
  if ((change.before & open) == 0) {
    return (change.after & open) == 0 && (previous_disabled & ~next_disabled) == 0;
  }
  if ((change.after & open) != 0) { return (next_disabled & ~previous_disabled) == 0; }
  return ((~previous_disabled & disabled) & ~next_disabled) == 0;
}
}

McuFlashConfig::McuFlashConfig(const FlashOptions options) {
  reset(options);
}

void McuFlashConfig::reset(const FlashOptions options) {
  options_ = options;
  divider_ = 0;
}

std::string_view McuFlashConfig::register_name(const std::uint16_t offset) {
  switch (offset) {
    case hc12::flash_clock_divider: return "FCLKDIV";
    case hc12::flash_security: return "FSEC";
    case hc12::flash_protection: return "FPROT";
    default: return {};
  }
}

std::uint8_t McuFlashConfig::read(const std::uint16_t offset) const {
  switch (offset) {
    case hc12::flash_clock_divider: return divider_;
    case hc12::flash_security: return options_.security;
    case hc12::flash_protection: return options_.protection;
    default: throw Fault("Unsupported MCU flash configuration read at " + hex(offset));
  }
}

void McuFlashConfig::write(const ByteWrite access) {
  switch (access.address) {
    case hc12::flash_clock_divider:
      if ((divider_ & loaded) == 0) { divider_ = static_cast<std::uint8_t>(access.value | loaded); }
      return;
    case hc12::flash_security: return; // Read-only; reset samples the nonvolatile field.
    case hc12::flash_protection: write_protection(access.value); return;
    default: throw Fault("Unsupported MCU flash configuration write at " + hex(access.address));
  }
}

void McuFlashConfig::unsecure() {
  options_.security = static_cast<std::uint8_t>((options_.security & ~3U) | 2U);
}

void McuFlashConfig::write_protection(const std::uint8_t value) {
  const auto previous = options_.protection;
  if (!allowed_scenario({.before = previous, .after = value})) { return; }
  unsigned frozen = 0;
  if ((previous & high_disabled) == 0) { frozen |= high_size; }
  if ((previous & low_disabled) == 0) { frozen |= low_size; }
  options_.protection = static_cast<std::uint8_t>((value & ~frozen) | (previous & frozen));
}

bool McuFlashConfig::secured() const {
  return (options_.security & low_size) != 2;
}

bool McuFlashConfig::backdoor_enabled() const {
  return (options_.security & key_mask) == loaded;
}

bool McuFlashConfig::protected_byte(const std::uint16_t physical_offset) const {
  if (physical_offset >= flash_bytes) {
    throw Fault("Invalid physical MCU flash offset " + hex(physical_offset));
  }
  const unsigned protection = options_.protection;
  const auto low_bytes = low_sector_bytes << (protection & low_size);
  const auto high_bytes = high_sector_bytes << ((protection & high_size) >> 3U);
  const bool selected =
      ((protection & low_disabled) == 0 && physical_offset < low_bytes) ||
      ((protection & high_disabled) == 0 && physical_offset >= flash_bytes - high_bytes);
  return (protection & open) != 0 ? selected : !selected;
}

bool McuFlashConfig::whole_array_unprotected() const {
  return (options_.protection & (open | disabled)) == (open | disabled);
}

std::optional<unsigned> McuFlashConfig::oscillator_divisor() const {
  if ((divider_ & loaded) == 0) { return {}; }
  return ((divider_ & prescale) != 0 ? 8U : 1U) * ((divider_ & divider_bits) + 1U);
}

}
