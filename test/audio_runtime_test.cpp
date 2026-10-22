// SPDX-License-Identifier: GPL-3.0-only
#include "audio_runtime_fixture.hpp"
#include "blofeld/audio_runtime.hpp"
#include "serial_pcm_fixture.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace blofeld::test::audio_runtime {
namespace {
void accepted(AudioRuntime &output, const unsigned index) {
  const auto frame = frame_at(index);
  wait_for([&] { return output.try_submit(frame) == PcmSubmission::accepted; }, output);
}

void finished(const AudioRuntime &output, const unsigned total) {
  const auto status = output.status();
  pcm::require(status.state == AudioRuntimeState::finished && status.accepted == total &&
                   status.written == total,
               "Closure must deliver the entire independently counted accepted prefix");
  std::cout << "accepted=" << status.accepted << " written=" << status.written
            << " underflows=" << status.underflows << '\n';
}

void short_or_tail(const std::string &server, const bool short_stream) {
  AudioRuntime output(options(server));
  const unsigned total = short_stream ? 1 : burst_frames;
  for (unsigned index = 0; index + 1 < total; ++index) { accepted(output, index); }
  if (!short_stream) {
    wait_for([&] { return output.status().written == total - 1; }, output);
  }
  accepted(output, total - 1);
  output.finish();
  finished(output, total);
  pcm::require(pcm::rejects([&] { output.check(); }).find("closed") != std::string::npos,
               "A completed stream cannot be rebound as an open PCM sink");
  pcm::require(output.try_submit(frame_at(total)) == PcmSubmission::closed,
               "A finished worker cannot accept and silently lose a new frame");
}

void paced(const std::string &server) {
  auto setup = options(server);
  constexpr unsigned queue_frames = 1021;
  setup.queue_frames = queue_frames;
  AudioRuntime output(setup);
  constexpr unsigned prefill = 4096;
  for (unsigned index = 0; index < prefill; ++index) { accepted(output, index); }
  wait_for([&] { return output.status().state == AudioRuntimeState::playing; }, output);
  const auto begin = std::chrono::steady_clock::now();
  constexpr std::uint64_t microseconds = 1000000;
  for (unsigned index = prefill; index < paced_frames; ++index) {
    const auto elapsed = (static_cast<std::uint64_t>(index - prefill) * microseconds) / sample_rate;
    std::this_thread::sleep_until(
        begin + std::chrono::microseconds{static_cast<std::chrono::microseconds::rep>(elapsed)});
    accepted(output, index);
  }
  output.finish();
  finished(output, paced_frames);
  pcm::require(output.status().underflows == 0,
               "Continuous accepted transport must complete without a hidden service underrun");
}
}
}

int main(const int count, char **values) {
  try {
    using namespace blofeld::test::audio_runtime;
    blofeld::test::pcm::require(count == 3 || count == 6,
                                "Audio runtime test needs mode/server and optional board inputs");
    const std::string mode(values[1]);
    const std::string server(values[2]);
    if (count == 6) {
      board_cases({.server = server,
                   .mode = mode,
                   .configuration = values[3],
                   .rom = values[4],
                   .flash = values[5]});
    } else if (mode == "short" || mode == "tail") {
      short_or_tail(server, mode == "short");
    } else if (mode == "paced") {
      paced(server);
    } else {
      failure_cases(server, mode);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
