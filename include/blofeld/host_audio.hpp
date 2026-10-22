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

// Blocking host transport, called outside CPU/peripheral execution and RT callbacks.
// Owns external playback state: never copy it into a board checkpoint or replay it.
class HostAudio {
public:
  static constexpr std::size_t maximum_block_frames = 256;
  HostAudio(SerialPcmFormat format, HostAudioClock clock, const HostAudioTarget &target = {});
  ~HostAudio();
  HostAudio(const HostAudio &) = delete;
  HostAudio &operator=(const HostAudio &) = delete;
  HostAudio(HostAudio &&) = delete;
  HostAudio &operator=(HostAudio &&) = delete;

  // Complete, contiguous stereo frames at the declared clock; no fill or resampling.
  // Validate the whole bounded block before transmitting any part of it.
  void write(std::span<const SerialPcmFrame> frames);
  void drain();
  [[nodiscard]] std::uint64_t frames_written() const;
  [[nodiscard]] bool faulted() const;

private:
  struct Stream;
  std::unique_ptr<Stream> stream_;
};
}
