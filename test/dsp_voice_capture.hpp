// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"
#include "dsp_norm_fixture.hpp"

#include <span>
#include <vector>

namespace blofeld::test {
// Offline immutable-per-stream delivery, outside modeled bus/receiver snapshots.
struct VoicePcmCapture {
  static constexpr unsigned stream_count = 32;
  static constexpr unsigned frames_per_stream = 128;
  using Stream = std::vector<SerialPcmFrame>;
  std::vector<Stream> streams;

  void begin() {
    require(streams.size() < stream_count, "Original audio capture stream bound");
    streams.emplace_back();
    streams.back().reserve(frames_per_stream);
  }

  void append(const std::span<const SerialPcmFrame> frames) {
    require(!streams.empty() && frames.size() <= frames_per_stream - streams.back().size(),
            "Original audio capture retains complete bounded streams");
    streams.back().insert(streams.back().end(), frames.begin(), frames.end());
  }
};
}
