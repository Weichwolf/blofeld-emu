// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/audio_runtime.hpp"
#include "blofeld/host_audio.hpp"
#include "blofeld/pcm_queue.hpp"
#include "blofeld/serial_pcm.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <stop_token>
#include <thread>

namespace blofeld {
struct AudioRuntime::Worker {
  static constexpr std::size_t batch_frames = 256;
  explicit Worker(AudioRuntimeOptions options);
  ~Worker();
  Worker(const Worker &) = delete;
  Worker &operator=(const Worker &) = delete;
  Worker(Worker &&) = delete;
  Worker &operator=(Worker &&) = delete;

  void run(const std::stop_token &stop) noexcept;
  void transfer(HostAudioOutput &output, const std::stop_token &stop);
  void refill();
  void check_progress(std::size_t admitted);
  void start_if_ready(HostAudioOutput &output, bool blocked, bool closing);
  [[nodiscard]] bool finish_if_ready(HostAudioOutput &output, bool closing);
  void publish(AudioRuntimeState value) noexcept;
  void observe(const HostAudioStatus &value) noexcept;
  void check_underflow(const HostAudioStatus &value);
  void check() const;
  [[nodiscard]] PcmSubmission submit(const SerialPcmFrame &frame) noexcept;

  const HostAudioConfig config;
  PcmQueue queue;
  std::array<SerialPcmFrame, batch_frames> batch{};
  std::size_t count = 0;
  std::size_t cursor = 0;
  std::optional<std::chrono::steady_clock::time_point> stalled_since;
  std::atomic<AudioRuntimeState> state{AudioRuntimeState::connecting};
  std::atomic<bool> closed{false};
  std::atomic<bool> retired{false};
  std::atomic<std::uint64_t> accepted{0};
  std::atomic<std::uint64_t> written{0};
  std::atomic<std::uint64_t> underflows{0};
  // Written once by the worker, published with failed/release, read with acquire.
  std::exception_ptr failure;
  // Destroy/join this last-created member before any state it can access.
  std::jthread thread;
};
}
