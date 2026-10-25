// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_wave.hpp"
#include "audio/serial_pcm.hpp"

#include <cstdint>
#include <filesystem>

namespace blofeld::test::pcm {
inline constexpr unsigned rate = 48000;
inline constexpr unsigned timebase = 30720000;
inline constexpr PcmWaveClock nominal{.timebase_hz = timebase, .sample_rate = rate};
inline constexpr unsigned first_tick = 100;
inline constexpr unsigned second_tick = 740;
inline constexpr unsigned payload_offset = 68;
inline constexpr unsigned valid_bits_offset = 38;
inline constexpr std::int32_t first_word = -5913697;
inline constexpr std::int32_t next_word = 1193046;

inline SerialPcmFrame frame(const std::uint64_t first = first_tick) {
  constexpr unsigned duration = 630;
  SerialPcmFrame result{
      .channels = 2, .first_tick = first, .last_tick = first + duration, .bit_ticks = 10};
  result.samples[0] = first_word;
  result.samples[1] = next_word;
  return result;
}

void wave_checks(const std::filesystem::path &directory);
}
