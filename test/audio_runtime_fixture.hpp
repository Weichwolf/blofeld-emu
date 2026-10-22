// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/audio_runtime.hpp"
#include "blofeld/serial_pcm.hpp"
#include "serial_pcm_fixture.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <thread>

namespace blofeld::test::audio_runtime {
inline constexpr unsigned sample_rate = 48000;
inline constexpr unsigned timebase = 30720000;
inline constexpr unsigned frame_period = 640;
inline constexpr unsigned frame_duration = 630;
inline constexpr unsigned first_tick = 100;
inline constexpr unsigned burst_frames = 1025;
inline constexpr unsigned paced_frames = 8192;
inline constexpr unsigned sample_cycle = 4093;
inline constexpr std::int32_t bias = 2046;
inline constexpr std::int32_t first_sample = -5913697;
inline constexpr std::int32_t next_sample = 1193046;

inline SerialPcmFrame frame_at(const unsigned index) {
  const auto start = first_tick + (static_cast<std::uint64_t>(index) * frame_period);
  SerialPcmFrame frame{
      .channels = 2, .first_tick = start, .last_tick = start + frame_duration, .bit_ticks = 10};
  frame.samples[0] =
      index == 0 ? first_sample : static_cast<std::int32_t>(index % sample_cycle) - bias;
  frame.samples[1] = index == 0 ? next_sample : -frame.samples[0];
  return frame;
}

inline AudioRuntimeOptions options(const std::string &server) {
  return {.output = {.timebase_hz = timebase, .sample_rate = sample_rate, .server = server}};
}

inline void wait_for(const auto &predicate, AudioRuntime &output) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
  while (!predicate()) {
    output.check();
    pcm::require(std::chrono::steady_clock::now() < deadline, "Audio worker did not make progress");
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
}

void closed_underrun(const std::string &server);
void failure_cases(const std::string &server, const std::string &mode);

struct BoardInputs {
  std::string server;
  std::string mode;
  std::filesystem::path configuration;
  std::filesystem::path rom;
  std::filesystem::path flash;
};

void board_cases(const BoardInputs &inputs);
}
