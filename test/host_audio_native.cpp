// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"
#include "dsp_pcm_native.hpp"
#include "host_audio_fixture.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld::test::audio {
std::vector<SerialPcmFrame> native_frames(const bool secondary) {
  constexpr std::uint64_t start = 10000;
  constexpr std::uint64_t baseline = 6500;
  constexpr std::uint64_t half_period = 125;
  constexpr std::uint64_t observations = (frame_count * 512) + 4;
  DspPcmRun run({.timebase = timebase,
                 .baseline = baseline,
                 .start = start,
                 .half_period = half_period,
                 .observations = observations,
                 .secondary = secondary});
  constexpr unsigned budget = 100000;
  const auto end = start + ((observations - 1) * half_period);
  run.advance_to(end, budget);
  const auto &frames = run.frames();
  pcm::require(frames.size() == frame_count && run.observed() == observations + 1,
               "Native ESAI must transmit every bounded test frame and consume its exact source");
  constexpr std::uint64_t first_sample = start + (10 * half_period);
  constexpr std::uint64_t bit_ticks = 8 * half_period;
  constexpr std::uint64_t bits_per_frame = 64;
  constexpr std::uint64_t frame_ticks = bits_per_frame * bit_ticks;
  constexpr std::uint64_t frame_span = (bits_per_frame - 1) * bit_ticks;
  for (std::size_t index = 0; index < frames.size(); ++index) {
    pcm::require(
        frames[index].samples[0] == (index == 0 ? first_word : next_word) &&
            frames[index].samples[1] == next_word &&
            frames[index].first_tick == first_sample + (index * frame_ticks) &&
            frames[index].last_tick == first_sample + (index * frame_ticks) + frame_span &&
            frames[index].bit_ticks == bit_ticks && frames[index].uniform_clock,
        "Native serial words and sample times must match the literal divider/frame ledger");
  }
  return frames;
}
}
