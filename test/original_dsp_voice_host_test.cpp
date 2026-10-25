// SPDX-License-Identifier: GPL-3.0-only
#include "audio/audio_runtime.hpp"
#include "common/bus.hpp"
#include "dsp_voice_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

int main(const int count, char **arguments) {
  try {
    const bool midi = count == 5 && std::string_view(arguments[4]) == "midi";
    const bool stream = count == 5 && std::string_view(arguments[4]) == "stream";
    if ((count != 4 && !midi && !stream) || !std::string_view(arguments[3]).starts_with("unix:")) {
      throw blofeld::Fault("Expected firmware, private trace and explicit private audio server");
    }
    // A declared host-playback rate for laboratory edge ordinals, not a
    // measured Blofeld HCKT/core/sample clock. Each serial bit spans eight ticks.
    constexpr std::uint32_t playback_rate = 48000;
    constexpr std::uint64_t reference_timebase = 24576000;
    constexpr unsigned repeating_frames = 96;
    constexpr unsigned finite_frames = 128;
    const unsigned expected_frames = stream ? repeating_frames : finite_frames;
    blofeld::AudioRuntime output({.output = {.pcm = {.frame_sync_high = false},
                                             .timebase_hz = reference_timebase,
                                             .sample_rate = playback_rate,
                                             .server = arguments[3]}});
    std::array<char *, 4> inputs{arguments[0], arguments[1], arguments[2], nullptr};
    auto input = blofeld::test::VoiceInput::none;
    if (stream) {
      input = blofeld::test::VoiceInput::stream;
    } else if (midi) {
      input = blofeld::test::VoiceInput::controllers;
    }
    const auto result = blofeld::test::run_voice_test(
        3, inputs.data(), false, true, !stream, {.direct = &output}, input);
    if (result != 0) { return result; }
    output.finish();
    const auto status = output.status();
    if (status.state != blofeld::AudioRuntimeState::finished ||
        status.accepted != expected_frames || status.written != expected_frames) {
      throw blofeld::Fault("Original serial source did not deliver its full accepted host prefix");
    }
    std::cout << "original-decoded-host-frames=" << status.written
              << " playback-rate=48000 physical-clock=unqualified\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
