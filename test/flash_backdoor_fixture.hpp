// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace blofeld::test::flash_key {
inline constexpr unsigned erased = 0xff;
inline constexpr unsigned word_mask = 0xffff;
inline constexpr unsigned nop = 0xa7;
inline constexpr unsigned key_offset = 0x7f00;
inline constexpr unsigned protection_offset = 0x7f0d;
inline constexpr unsigned security_offset = 0x7f0f;
inline constexpr unsigned idle_status = 0xc0;
inline constexpr unsigned interrupt_enable = 0xc0;
inline constexpr unsigned all_control = 0xe0;
inline constexpr unsigned keyen_enabled = 0x80;
inline constexpr unsigned keyen_enabled_end = 0xc0;
inline constexpr unsigned retained_security_mask = 0xfc;
inline constexpr unsigned flash_base = 0x8000;
inline constexpr unsigned test_target = 0x8234;
inline constexpr unsigned test_offset = 0x234;
inline constexpr unsigned test_data = 0x1234;
inline constexpr unsigned divider = 0x100;
inline constexpr unsigned divider_value = 0x48;
inline constexpr unsigned command = 0x106;
inline constexpr unsigned word_program = 0x20;
inline constexpr unsigned buffer_empty = 0x80;
inline constexpr unsigned protected_status = 0xe0;
inline constexpr unsigned oscillator_hz = 12000000;
inline constexpr unsigned bus_hz = 24000000;
inline constexpr unsigned diagnostic_pc = 0x1802;
inline constexpr unsigned masked_flags = 0xd0;
inline constexpr unsigned first_key_high_byte = 0x13;
inline constexpr unsigned control = 0x103;
inline constexpr unsigned security = 0x101;
inline constexpr unsigned protection = 0x104;
inline constexpr unsigned status = 0x105;
inline constexpr unsigned key_access = 0x20;
inline constexpr unsigned first_key = 0xff00;
inline constexpr std::uint8_t secured = 0xbc;
inline constexpr std::uint8_t unsecured = 0xbe;
inline constexpr std::array<unsigned, 4> keys{0x1357, 0x2468, 0x5a3c, 0x8e71};

inline void require(const bool value, const std::string_view message) {
  if (!value) { throw Fault(std::string(message)); }
}

void reject(const auto &operation, const std::string_view expected) {
  try {
    operation();
  } catch (const Fault &fault) {
    require(std::string_view(fault.what()).contains(expected),
            "Key-access fault retains its cause");
    return;
  }
  throw Fault("Expected rejected key-access operation");
}

McuFlashImage image(std::uint8_t options = secured);
void put(McuBus &bus, unsigned index, unsigned word);
void feed(McuBus &bus);
void unchanged(const McuBus &bus, const McuFlashImage &before);
void test_native_keys_and_partial_owner_restore();
}
