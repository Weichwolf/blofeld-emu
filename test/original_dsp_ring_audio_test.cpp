// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/audio_runtime.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/pcm_sink.hpp"
#include "dsp_voice_fixture.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <string_view>

namespace {
unsigned selected_case(const std::string_view text) {
  if (text.size() != 1 || text.front() < '0' || text.front() > '7') {
    throw blofeld::Fault("Expected one original ring context index from 0 through 7");
  }
  return static_cast<unsigned>(text.front() - '0');
}

void reject_other_source(blofeld::PcmFrameSink &output) {
  bool refused = false;
  try {
    output.claim_source(std::make_shared<const bool>(false));
  } catch (const blofeld::Fault &error) {
    refused = std::string_view(error.what()).contains("belongs to another board");
  }
  if (!refused) { throw blofeld::Fault("Original ring output did not retain its source identity"); }
  output.check();
}
}

int main(const int count, char **arguments) {
  try {
    if ((count != 4 && count != 5) || !std::string_view(arguments[3]).starts_with("unix:")) {
      throw blofeld::Fault("Expected firmware, private trace and explicit private audio server");
    }
    const auto index = selected_case(count == 5 ? arguments[4] : "0");
    // Declared playback rate for laboratory edge ordinals; not a physical clock.
    constexpr unsigned playback_rate = 48000;
    constexpr unsigned reference_timebase = 24576000;
    constexpr unsigned expected_frames = 96;
    blofeld::AudioRuntime output({.output = {.pcm = {.frame_sync_high = false},
                                             .timebase_hz = reference_timebase,
                                             .sample_rate = playback_rate,
                                             .server = arguments[3]}});
    const auto result =
        blofeld::test::run_voice_test(3,
                                      arguments,
                                      false,
                                      true,
                                      false,
                                      {.direct = &output, .stream_case_index = index},
                                      blofeld::test::VoiceInput::stream);
    if (result != 0) { return result; }
    reject_other_source(output);
    output.finish();
    const auto status = output.status();
    if (status.state != blofeld::AudioRuntimeState::finished ||
        status.accepted != expected_frames || status.written != expected_frames) {
      throw blofeld::Fault("Original DMA ring did not deliver its complete native host prefix");
    }
    std::cout << "direct-ring-frames=" << status.written
              << " playback-rate=48000 physical-clock=unqualified source-reuse-rejected=1\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
