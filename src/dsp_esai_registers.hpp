// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::esai {
inline constexpr unsigned channels = 0x3f;
inline constexpr unsigned tsr = 6;
inline constexpr unsigned rx_first = 8;
inline constexpr unsigned rx_last = 11;
inline constexpr unsigned saisr = 19;
inline constexpr unsigned saicr = 20;
inline constexpr unsigned tcr = 21;
inline constexpr unsigned tccr = 22;
inline constexpr unsigned rcr = 23;
inline constexpr unsigned rccr = 24;
inline constexpr unsigned tsma = 25;
inline constexpr unsigned tsmb = 26;
inline constexpr unsigned rsma = 27;
inline constexpr unsigned rsmb = 28;
inline constexpr unsigned shift_lsb = 0x40;
inline constexpr unsigned right_align = 0x80;
inline constexpr unsigned network = 0x100;
inline constexpr unsigned bit_sync = 0x8000;
inline constexpr unsigned zero_pad = 0x20000;
inline constexpr unsigned personal_reset = 0x80000;
inline constexpr unsigned clock_polarity = 0x40000;
inline constexpr unsigned frame_polarity = 0x80000;
inline constexpr unsigned alignment = 0x100;
inline constexpr unsigned frame_flag = 0x2000;
inline constexpr unsigned underrun = 0x4000;
inline constexpr unsigned empty = 0x8000;
inline constexpr unsigned even_empty = 0x10000;
inline constexpr unsigned odd_empty = 0x20000;
inline constexpr unsigned empty_flags = empty | even_empty | odd_empty;
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr unsigned half_mask = 0xffff;
}
