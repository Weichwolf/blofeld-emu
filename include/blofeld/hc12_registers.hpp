// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

// MC9S12UF32 register offsets at the reset register-block base.
namespace blofeld::hc12 {
inline constexpr std::uint16_t reset_sp = 0x00ff;
inline constexpr std::uint8_t reset_ccr = 0xd0;
inline constexpr std::uint16_t reset_vector = 0xfffe;
inline constexpr std::uint32_t address_mask = 0xffff;
inline constexpr std::uint16_t ddra = 0x0002;
inline constexpr std::uint16_t ddrb = 0x0003;
inline constexpr std::uint16_t porte = 0x0008;
inline constexpr std::uint16_t ddre = 0x0009;
inline constexpr std::uint16_t pear = 0x000a;
inline constexpr std::uint16_t initee = 0x0012;
inline constexpr std::uint16_t tscr1 = 0x0046;
inline constexpr std::uint16_t ptt = 0x0240;
inline constexpr std::uint16_t ddrt = 0x0242;
inline constexpr std::uint16_t ptm = 0x0248;
inline constexpr std::uint16_t ddrm = 0x024a;
inline constexpr std::uint16_t ptj = 0x0250;
inline constexpr std::uint16_t ddrj = 0x0252;
inline constexpr std::uint16_t modrr = 0x0257;
inline constexpr std::uint16_t ptq = 0x0260;
inline constexpr std::uint16_t ddrq = 0x0262;
inline constexpr std::uint16_t perq = 0x0264;
inline constexpr std::uint16_t ppsq = 0x0265;
}
