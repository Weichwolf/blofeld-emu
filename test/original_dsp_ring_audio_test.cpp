// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/audio_runtime.hpp"
#include "blofeld/bus.hpp"
#include "dsp_voice_fixture.hpp"

#include <exception>
#include <iostream>
#include <string_view>

int main(const int count, char **arguments) {
  try {
    const bool midi = count == 5 && std::string_view(arguments[4]) == "midi";
    if ((count != 4 && !midi) || !std::string_view(arguments[3]).starts_with("unix:")) {
      throw blofeld::Fault("Expected firmware, private trace and explicit private audio server");
    }
    // Declared playback rate for laboratory edge ordinals; not a physical clock.
    constexpr unsigned playback_rate = 48000;
    constexpr unsigned reference_timebase = 24576000;
    constexpr unsigned unchanged_frames = 96;
    constexpr unsigned midi_frames = 160;
    const auto expected_frames = midi ? midi_frames : unchanged_frames;
    blofeld::AudioRuntime output({.output = {.pcm = {.frame_sync_high = false},
                                             .timebase_hz = reference_timebase,
                                             .sample_rate = playback_rate,
                                             .server = arguments[3]}});
    const auto input =
        midi ? blofeld::test::VoiceInput::stream_midi : blofeld::test::VoiceInput::stream;
    const auto result =
        blofeld::test::run_voice_test(3, arguments, false, true, false, {.direct = &output}, input);
    if (result != 0) { return result; }
    output.finish();
    const auto status = output.status();
    if (status.state != blofeld::AudioRuntimeState::finished ||
        status.accepted != expected_frames || status.written != expected_frames) {
      throw blofeld::Fault("Original DMA ring did not deliver its complete native host prefix");
    }
    std::cout << "direct-ring-frames=" << status.written
              << " playback-rate=48000 physical-clock=unqualified\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
