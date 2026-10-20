// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::dma {
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr unsigned status = 0xfffff4;
inline constexpr unsigned first_offset = 0xfffff3;
inline constexpr unsigned first_source = 0xffffef;
inline constexpr unsigned last_control = 0xffffd8;
inline constexpr unsigned enable = 0x800000;
inline constexpr unsigned interrupt = 0x400000;
inline constexpr unsigned continuous = 0x10000;
inline constexpr unsigned mode_shift = 19;
inline constexpr unsigned priority_shift = 17;
inline constexpr unsigned request_shift = 11;
inline constexpr unsigned request_mask = 31;
inline constexpr unsigned three_dimensions = 0x400;
inline constexpr unsigned address_shift = 4;
inline constexpr unsigned address_mask = 63;
inline constexpr unsigned known_source = 1;
inline constexpr unsigned known_destination = 2;
inline constexpr unsigned known_counter = 4;
inline constexpr unsigned known_all = 7;
inline constexpr unsigned settling_cycles = 3;
}
