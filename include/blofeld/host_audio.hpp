// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace blofeld {
struct HostAudioClock {
  std::uint64_t timebase_hz = 0;
  std::uint32_t sample_rate = 0;
};

struct HostAudioTarget {
  // Empty selects the session server/default sink; no host routing is changed.
  std::string server;
  std::string sink;
};

struct HostAudioBoundary {
  std::shared_ptr<const void> owner;
  std::uint64_t restore_revision = 0;
  std::uint64_t ticks = 0;
};

// Nonblocking single-owner transport; poll outside native execution and RT callbacks.
// Owns external playback state: never copy it into a board checkpoint or replay it.
class HostAudio {
public:
  static constexpr std::size_t maximum_block_frames = 256;
  static constexpr std::size_t queue_capacity = 4096;
  HostAudio(SerialPcmFormat format, HostAudioClock clock, const HostAudioTarget &target = {});
  ~HostAudio();
  HostAudio(const HostAudio &) = delete;
  HostAudio &operator=(const HostAudio &) = delete;
  HostAudio(HostAudio &&) = delete;
  HostAudio &operator=(HostAudio &&) = delete;

  // Complete, contiguous stereo frames at the declared clock; no fill or resampling.
  // Validate the whole block before enqueueing; no host I/O, allocation or waiting.
  // Queue exhaustion rejects without dropping the accepted prefix.
  void write(std::span<const SerialPcmFrame> frames);
  void poll(const HostAudioBoundary &boundary);
  void finish();
  void retire();
  [[nodiscard]] std::uint64_t frames_written() const;
  [[nodiscard]] std::size_t available_frames() const;
  [[nodiscard]] std::size_t queued_frames() const;
  [[nodiscard]] bool ready() const;
  [[nodiscard]] bool finished() const;
  [[nodiscard]] bool faulted() const;

private:
  struct Stream;
  std::unique_ptr<Stream> stream_;
};
}
