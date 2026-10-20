// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::dax {
inline constexpr unsigned control = 0xffffd0;
inline constexpr unsigned non_audio = 0xffffd1;
inline constexpr unsigned audio_a = 0xffffd2;
inline constexpr unsigned audio_b = 0xffffd3;
inline constexpr unsigned status = 0xffffd4;
inline constexpr unsigned non_audio_mask = 0xfc00;
inline constexpr unsigned empty = 1;
inline constexpr unsigned underrun = 2;
inline constexpr unsigned last_block_frame = 4;
inline constexpr unsigned start_block = 0x20;
inline constexpr unsigned clock_select = 0x18;
inline constexpr unsigned frame_units = 128;
inline constexpr unsigned subframe_units = 64;
inline constexpr unsigned preamble_units = 8;
inline constexpr unsigned frames_per_block = 192;
inline constexpr unsigned audio_bits = 24;
inline constexpr unsigned payload_bits = 27;
inline constexpr unsigned preamble_x = 0x1d;
inline constexpr unsigned preamble_y = 0x1b;
inline constexpr unsigned preamble_z = 0x17;
}
