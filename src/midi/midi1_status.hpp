// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>

namespace blofeld::detail::midi1 {
inline constexpr std::uint8_t note_off = 0x80, note_on = 0x90, key_pressure = 0xa0;
inline constexpr std::uint8_t control_change = 0xb0, program_change = 0xc0, channel_pressure = 0xd0;
inline constexpr std::uint8_t pitch_bend = 0xe0, sysex_start = 0xf0, quarter_frame = 0xf1;
inline constexpr std::uint8_t song_position = 0xf2, song_select = 0xf3, tune_request = 0xf6;
inline constexpr std::uint8_t sysex_end = 0xf7, clock = 0xf8, start = 0xfa, resume = 0xfb;
inline constexpr std::uint8_t stop = 0xfc, active_sensing = 0xfe, reset = 0xff;
inline constexpr unsigned family_mask = 0xf0, data_entry_fine = 38, last_coarse_controller = 31;
}
