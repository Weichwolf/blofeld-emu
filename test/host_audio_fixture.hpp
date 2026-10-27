// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld::test::audio {
inline constexpr std::uint64_t timebase = 3072000000;
inline constexpr std::uint32_t rate = 48000;
inline constexpr std::size_t frame_count = 256;
inline constexpr std::int32_t first_word = -5913697;
inline constexpr std::int32_t next_word = 1193046;
[[nodiscard]] std::vector<SerialPcmFrame> native_frames(bool secondary);
void negotiation(const std::string &server);
}
