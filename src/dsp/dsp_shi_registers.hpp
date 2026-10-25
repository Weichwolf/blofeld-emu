// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld::shi {
inline constexpr std::uint32_t hckr = 0xffff90;
inline constexpr std::uint32_t hcsr = 0xffff91;
inline constexpr std::uint32_t hsar = 0xffff92;
inline constexpr std::uint32_t htx = 0xffff93;
inline constexpr std::uint32_t hrx = 0xffff94;
inline constexpr unsigned word_bits = 24;
inline constexpr std::uint32_t word_mask = 0xffffff;
inline constexpr std::uint32_t top_bit = 0x800000;
inline constexpr std::uint32_t hen = 1;
inline constexpr std::uint32_t i2c = 2;
inline constexpr std::uint32_t fifo = 0x20;
inline constexpr std::uint32_t master = 0x40;
inline constexpr std::uint32_t idle = 0x200;
inline constexpr std::uint32_t interrupt_mask = 0x3c00;
inline constexpr std::uint32_t underrun = 0x4000;
inline constexpr std::uint32_t transmit_empty = 0x8000;
inline constexpr std::uint32_t receive_not_empty = 0x020000;
inline constexpr std::uint32_t receive_full = 0x080000;
inline constexpr std::uint32_t overrun = 0x100000;
inline constexpr std::uint32_t busy = 0x400000;
inline constexpr std::uint32_t control_mask = 0x3fff;
}
