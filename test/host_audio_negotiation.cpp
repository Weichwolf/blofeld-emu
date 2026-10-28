// SPDX-License-Identifier: GPL-3.0-only
#include "audio/host_audio.hpp"
#include "common/bus.hpp"
#include "host_audio_fixture.hpp"
#include "serial_pcm_fixture.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace blofeld::test::audio {
namespace {
void report_stream(const HostAudioOutput &output, const unsigned preferred, const unsigned target) {
  const auto info = output.stream_info();
  pcm::require(info.format == HostAudioPcmFormat::stereo_signed32_native &&
                   info.sample_rate == rate && info.channels == 2 && info.maximum_frames != 0 &&
                   info.target_frames <= info.maximum_frames && info.target_frames != 0 &&
                   info.prebuffer_frames <= info.target_frames &&
                   info.minimum_request_frames <= info.target_frames,
               "The negotiated client stream retains the independent source PCM contract");
  std::cout << "{\"requested_frames\":" << preferred << ",\"requested_target_frames\":" << target
            << ",\"rate\":" << info.sample_rate << ",\"channels\":" << info.channels
            << R"(,"format":"s32ne","maximum":)" << info.maximum_frames
            << ",\"target\":" << info.target_frames << ",\"prebuffer\":" << info.prebuffer_frames
            << ",\"minimum_request\":" << info.minimum_request_frames
            << ",\"index\":" << info.stream_index << ",\"device\":";
  if (info.device_index) {
    std::cout << *info.device_index;
  } else {
    std::cout << "null";
  }
  std::cout << ",\"suspended\":";
  if (info.suspended) {
    std::cout << (*info.suspended ? "true" : "false");
  } else {
    std::cout << "null";
  }
  std::cout << "}\n" << std::flush;
}

void observe_control(HostAudioOutput &output, std::istringstream &command) {
  std::uint32_t device = 0;
  unsigned suspended = 0;
  pcm::require(static_cast<bool>(command >> device >> suspended) && suspended <= 1,
               "The owner supplies an actual private sink index and suspend state");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  for (;;) {
    output.poll();
    const auto info = output.stream_info();
    if (info.device_index == device && info.suspended == (suspended != 0)) { return; }
    pcm::require(std::chrono::steady_clock::now() < deadline,
                 "The actual move/suspend notification must become observable");
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
}
}

void negotiation(const std::string &server) {
  unsigned preferred = 0;
  unsigned target = 0;
  std::string line;
  std::getline(std::cin, line);
  std::istringstream preferences(line);
  pcm::require(static_cast<bool>(preferences >> preferred),
               "The owner supplies a buffer preference");
  if (!preferences.eof()) {
    pcm::require(static_cast<bool>(preferences >> target), "Invalid separate target preference");
  }
  HostAudioOutput output({.timebase_hz = timebase,
                          .sample_rate = rate,
                          .buffer_frames = preferred,
                          .target_frames = target,
                          .server = server});
  report_stream(output, preferred, target);
  while (std::getline(std::cin, line)) {
    std::istringstream command(line);
    std::string operation;
    command >> operation;
    if (operation == "observe") {
      observe_control(output, command);
      pcm::require(output.status().frames_written == 0,
                   "Host routing controls cannot manufacture source audio");
      report_stream(output, preferred, target);
    } else {
      pcm::require(operation == "complete", "Unknown directed host stream control");
      const auto frames = native_frames(false);
      pcm::require(output.write(frames) == frame_count,
                   "The measured private buffer admits actual native DSP/ESAI frames");
      output.start();
      output.drain();
      pcm::require(output.status().frames_written == frame_count && output.status().drained,
                   "Playback completes its source prefix after host-only routing changes");
      std::cout << "frames=" << output.status().frames_written << '\n';
      return;
    }
  }
  throw Fault("The negotiation owner disconnected before actual playback");
}
}
