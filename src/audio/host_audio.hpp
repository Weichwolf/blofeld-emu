// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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
  // Server stream byte position, not the client's callback arrival position.
  // -1 retains unknownness; the earliest event survives subsequent underruns.
  std::int64_t earliest_underflow_byte = -1;
  bool started = false;
  bool drained = false;
};

enum class HostAudioPcmFormat : std::uint8_t { stereo_signed32_native };

// Last validated client stream, not the physical sink's sample clock/format.
// Buffer dimensions are actual grants in complete stereo frames. Unsupported
// optional server metadata remains unknown. Refresh belongs to ordinary poll.
struct HostAudioStreamInfo {
  HostAudioPcmFormat format = HostAudioPcmFormat::stereo_signed32_native;
  std::uint32_t sample_rate = 0;
  std::uint32_t channels = 0;
  std::uint32_t maximum_frames = 0;
  std::uint32_t target_frames = 0;
  std::uint32_t prebuffer_frames = 0;
  std::uint32_t minimum_request_frames = 0;
  std::uint32_t stream_index = 0;
  std::optional<std::uint32_t> device_index;
  std::optional<bool> suspended;
  bool operator==(const HostAudioStreamInfo &) const = default;
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
  [[nodiscard]] HostAudioStreamInfo stream_info() const;

private:
  struct Session;
  std::unique_ptr<Session> session_;
};
}
