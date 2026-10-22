// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace blofeld {
struct HostAudioConfig {
  SerialPcmFormat pcm{};
  std::uint64_t timebase_hz = 0;
  std::uint32_t sample_rate = 0;
  std::uint32_t buffer_frames = 4096;
  std::chrono::milliseconds timeout = std::chrono::seconds{2};
  std::string server;
};

struct HostAudioStatus {
  std::uint64_t frames_written = 0;
  std::uint64_t underflows = 0;
  bool started = false;
  bool drained = false;
};

// Single ordinary host-thread adapter, separate from board execution. No RT
// callback or host synthesis. The caller retains frames when write returns 0.
// Empty server selects the normal session server and its default output.
class HostAudioOutput {
public:
  explicit HostAudioOutput(HostAudioConfig config);
  ~HostAudioOutput();
  HostAudioOutput(const HostAudioOutput &) = delete;
  HostAudioOutput &operator=(const HostAudioOutput &) = delete;
  HostAudioOutput(HostAudioOutput &&) = delete;
  HostAudioOutput &operator=(HostAudioOutput &&) = delete;

  [[nodiscard]] std::size_t write(std::span<const SerialPcmFrame> frames);
  void start();
  void poll();
  void drain();
  [[nodiscard]] HostAudioStatus status() const;

private:
  struct Session;
  std::unique_ptr<Session> session_;
};
}
