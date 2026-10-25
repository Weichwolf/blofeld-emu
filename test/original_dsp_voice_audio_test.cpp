// SPDX-License-Identifier: GPL-3.0-only
#include "audio/audio_runtime.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_voice_capture.hpp"
#include "dsp_voice_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace blofeld::test {
namespace {
void deliver(const VoicePcmCapture &capture, const std::string &server) {
  require(capture.streams.size() == VoicePcmCapture::stream_count,
          "Original native audio matrix must finish before external delivery");
  std::cout << "AUDIO_READY\n" << std::flush;
  std::string command;
  require(static_cast<bool>(std::getline(std::cin, command)) && command == "play",
          "Private monitor must acknowledge readiness before playback");
  constexpr unsigned sample_rate = 48000;
  constexpr unsigned frame_bits = 64;
  constexpr unsigned bit_ticks = 8;
  constexpr std::uint64_t timebase = std::uint64_t{sample_rate} * frame_bits * bit_ticks;
  for (unsigned index = 0; index < capture.streams.size(); ++index) {
    const auto &stream = capture.streams.at(index);
    require(stream.size() == VoicePcmCapture::frames_per_stream,
            "Every native capture must retain all decoded frames");
    AudioRuntime output({.output = {.pcm = {.frame_sync_high = false},
                                    .timebase_hz = timebase,
                                    .sample_rate = sample_rate,
                                    .server = server}});
    for (const auto &frame : stream) { output.accept(frame); }
    output.finish();
    const auto status = output.status();
    require(status.state == AudioRuntimeState::finished && status.accepted == stream.size() &&
                status.written == stream.size(),
            "Actual worker closure must deliver every original decoded frame");
    std::cout << "AUDIO_STREAM " << index << ' ' << status.accepted << ' ' << status.written << ' '
              << status.underflows << '\n';
  }
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 4,
                           "Expected original firmware, private trace and private server");
    blofeld::test::VoicePcmCapture capture;
    const auto result =
        blofeld::test::run_voice_test(3, arguments, false, true, true, {.capture = &capture});
    if (result != 0) { return result; }
    blofeld::test::deliver(capture, arguments[3]);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
