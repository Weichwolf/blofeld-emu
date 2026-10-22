// SPDX-License-Identifier: GPL-3.0-only
#include "audio_runtime_fixture.hpp"
#include "blofeld/audio_runtime.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/serial_pcm.hpp"
#include "serial_pcm_fixture.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace blofeld::test::audio_runtime {
namespace {
void bad_or_empty(const std::string &server, const bool empty) {
  AudioRuntime output(options(server));
  if (!empty) {
    auto invalid = frame_at(0);
    invalid.uniform_clock = false;
    output.accept(invalid);
  }
  const auto failure = pcm::rejects([&] { output.finish(); });
  pcm::require(failure.find(empty ? "without a complete PCM" : "shape/ticks") != std::string::npos,
               "Invalid/absent source frames must not become a successful audio stream");
  pcm::require(output.status().state == AudioRuntimeState::failed && output.status().written == 0,
               "A failed source cannot publish a fabricated audio prefix");
  pcm::require(output.try_submit(frame_at(0)) == PcmSubmission::failed,
               "An asynchronous failure rejects future producer submissions");
}

void retirement_and_cancellation(const std::string &server, const bool cancel) {
  AudioRuntime output(options(server));
  output.accept(frame_at(0));
  if (cancel) { return; }
  output.invalidate();
  pcm::require(output.try_submit(frame_at(1)) == PcmSubmission::retired,
               "Invalidation immediately closes producer admission");
  pcm::require(pcm::rejects([&] { output.finish(); }).find("retired") != std::string::npos &&
                   output.status().state == AudioRuntimeState::retired,
               "Retirement cannot be mistaken for successful drain");
}

void underrun(const std::string &server) {
  auto setup = options(server);
  constexpr unsigned prefill = 256;
  setup.output.buffer_frames = prefill;
  AudioRuntime output(setup);
  for (unsigned index = 0; index < prefill; ++index) { output.accept(frame_at(index)); }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
  while (output.status().state != AudioRuntimeState::failed) {
    pcm::require(std::chrono::steady_clock::now() < deadline, "Expected real service starvation");
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  pcm::require(pcm::rejects([&] { output.finish(); }).find("underrun") != std::string::npos &&
                   output.status().underflows != 0,
               "A starved running source must visibly fail instead of accepting service silence");
}

void service_failure(const std::string &server, const bool stopped) {
  AudioRuntime output(options(server));
  std::cout << "READY\n" << std::flush;
  std::string release;
  pcm::require(static_cast<bool>(std::getline(std::cin, release)) && release == "continue",
               "The parent must release the directed service-failure case");
  const auto failure = pcm::rejects([&] {
    output.accept(frame_at(0));
    output.finish();
  });
  pcm::require(failure.find(stopped ? "deadline expired" : "Host audio") != std::string::npos,
               "Worker service loss/timeouts retain the native client's diagnostic");
}

void suspended_sink(const std::string &server, const bool pending_write) {
  auto setup = options(server);
  constexpr unsigned service_frames = 256;
  setup.output.buffer_frames = service_frames;
  AudioRuntime output(setup);
  const auto total = pending_write ? burst_frames : 1U;
  for (unsigned index = 0; index < total; ++index) { output.accept(frame_at(index)); }
  const auto begin = std::chrono::steady_clock::now();
  const auto failure = pcm::rejects([&] { output.finish(); });
  pcm::require(failure.find(pending_write ? "write progress deadline" : "drain deadline") !=
                   std::string::npos,
               "A responsive service with a suspended sink must bound write/drain stalls");
  pcm::require(std::chrono::steady_clock::now() - begin < std::chrono::seconds{5} &&
                   output.status().state == AudioRuntimeState::failed &&
                   (pending_write ? output.status().written < total : output.status().written == 1),
               "Shutdown cannot wait forever for a reachable service that stops consuming PCM");
}
}

void failure_cases(const std::string &server, const std::string &mode) {
  if (mode == "closed-underrun") {
    closed_underrun(server);
  } else if (mode == "invalid" || mode == "empty") {
    bad_or_empty(server, mode == "empty");
  } else if (mode == "retire" || mode == "cancel") {
    retirement_and_cancellation(server, mode == "cancel");
  } else if (mode == "underrun") {
    underrun(server);
  } else if (mode == "death" || mode == "stopped") {
    service_failure(server, mode == "stopped");
  } else if (mode == "blocked-write" || mode == "blocked-drain") {
    suspended_sink(server, mode == "blocked-write");
  } else if (mode == "missing" || mode == "timeout") {
    auto setup = options(server);
    constexpr std::chrono::milliseconds bound{50};
    setup.output.timeout = bound;
    const auto failure = pcm::rejects([&] { const AudioRuntime invalid(setup); });
    pcm::require(failure.find("Host audio") != std::string::npos,
                 "Failed worker construction must propagate its original exception");
  } else {
    throw Fault("Unknown directed audio worker test mode");
  }
}
}
