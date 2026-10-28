// SPDX-License-Identifier: GPL-3.0-only
#include "audio/host_audio.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "host_audio_fixture.hpp"
#include "serial_pcm_fixture.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace blofeld::test::audio {
namespace {
HostAudioConfig config(const std::string &server) {
  return {
      .timebase_hz = timebase, .sample_rate = rate, .buffer_frames = frame_count, .server = server};
}

void invalid_configuration(const std::string &server) {
  for (unsigned variant = 0; variant < 9; ++variant) {
    auto setup = config(server);
    switch (variant) {
      case 0: setup.pcm.channels = 1; break;
      case 1: setup.pcm.word_bits = 0; break;
      case 2: setup.timebase_hz = 1; break;
      case 3: setup.sample_rate = 0; break;
      case 4: setup.buffer_frames = 1; break;
      case 5: setup.timeout = std::chrono::milliseconds{0}; break;
      case 6: setup.server.push_back('\0'); break;
      case 7: ++setup.timebase_hz; break;
      case 8: setup.target_frames = setup.buffer_frames + 1; break;
      default: throw Fault("Unknown fixed audio configuration control");
    }
    (void)pcm::rejects([&] { const HostAudioOutput invalid(setup); });
  }
}

void invalid_frames(HostAudioOutput &output, const std::vector<SerialPcmFrame> &frames) {
  for (unsigned variant = 0; variant < 7; ++variant) {
    auto invalid = frames;
    auto &frame = invalid.back();
    switch (variant) {
      case 0: frame.channels = 1; break;
      case 1: frame.uniform_clock = false; break;
      case 2: frame.bit_ticks.reset(); break;
      case 3: ++frame.last_tick; break;
      case 4: --frame.first_tick; break;
      case 5: frame.samples[0] = std::numeric_limits<std::int32_t>::max(); break;
      case 6: invalid.push_back(frame); break;
      default: throw Fault("Unknown fixed audio frame control");
    }
    (void)pcm::rejects([&] { (void)output.write(invalid); });
    pcm::require(output.status().frames_written == 0,
                 "Invalid final frames must not publish an otherwise valid prefix");
  }
}

void positive(const std::string &server, const bool secondary) {
  const auto frames = native_frames(secondary);
  HostAudioOutput output(config(server));
  (void)pcm::rejects([&] { output.start(); });
  (void)pcm::rejects([&] { output.drain(); });
  invalid_frames(output, frames);
  pcm::require(output.write(frames) == frame_count, "A corked server admits its bounded buffer");
  auto next = frames.back();
  const auto period = timebase / rate;
  next.first_tick += period;
  next.last_tick += period;
  pcm::require(output.write(std::span(&next, 1)) == 0,
               "A full corked server returns backpressure without dropping an input frame");
  output.start();
  (void)pcm::rejects([&] { output.start(); });
  output.drain();
  (void)pcm::rejects([&] { (void)output.write(std::span(&next, 1)); });
  (void)pcm::rejects([&] { output.drain(); });
  const auto status = output.status();
  pcm::require(status.started && status.drained && status.frames_written == frame_count,
               "The actual service must acknowledge start, all frames and completed drain");
  std::cout << "frames=" << status.frames_written << " underflows=" << status.underflows << '\n';
}

void wait_control() {
  std::cout << "READY\n" << std::flush;
  std::string control;
  pcm::require(static_cast<bool>(std::getline(std::cin, control)) && control == "continue",
               "The parent must explicitly release the fixed service failure test");
}

void connection_failure(const std::string &server, const std::string_view mode) {
  auto setup = config(server);
  constexpr std::chrono::milliseconds bound{50};
  setup.timeout = bound;
  const auto begin = std::chrono::steady_clock::now();
  const auto failure = pcm::rejects([&] { const HostAudioOutput invalid(setup); });
  pcm::require(failure.find("Host audio") != std::string::npos &&
                   (mode != "timeout" || failure.find("deadline expired") != std::string::npos),
               "Missing and stalled servers must visibly fail in the host adapter");
  pcm::require(std::chrono::steady_clock::now() - begin < std::chrono::seconds{1},
               "A stalled service cannot block setup indefinitely");
}

void service_failure(const std::string &server, const bool stopped) {
  auto setup = config(server);
  HostAudioOutput output(setup);
  const auto frames = native_frames(false);
  pcm::require(output.write(frames) == frame_count, "The failure test queues actual native PCM");
  wait_control();
  const auto failure = pcm::rejects([&] {
    if (stopped) { output.start(); }
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds{3};
    while (std::chrono::steady_clock::now() < end) {
      output.poll();
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  });
  pcm::require(failure.find(stopped ? "deadline expired" : "Host audio") != std::string::npos,
               "Service death or a stopped service must produce a visible bounded failure");
  pcm::require(pcm::rejects([&] { output.poll(); }).find("new session") != std::string::npos,
               "A transport fault cannot silently resume the same session");
}
}
}

int main(const int count, char **values) {
  try {
    using namespace blofeld::test::audio;
    blofeld::test::pcm::require(count == 3, "Host audio test needs a mode and a private server");
    const std::string mode(values[1]);
    const std::string server(values[2]);
    if (mode == "primary" || mode == "secondary" || mode == "default") {
      invalid_configuration(server);
      positive(mode == "default" ? "" : server, mode == "secondary");
    } else if (mode == "negotiation") {
      negotiation(server);
    } else if (mode == "missing" || mode == "timeout") {
      connection_failure(server, mode);
    } else if (mode == "death" || mode == "stopped") {
      service_failure(server, mode == "stopped");
    } else {
      throw blofeld::Fault("Unknown directed host audio test mode");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
