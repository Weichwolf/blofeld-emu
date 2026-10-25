// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"

#include <cstdint>
#include <exception>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::pcm {
inline void require(const bool condition,
                    const std::string_view message,
                    const std::source_location where = std::source_location::current()) {
  if (!condition) {
    throw Fault(std::string(message) + " at line " + std::to_string(where.line()));
  }
}

inline std::string rejects(const auto &action) {
  try {
    action();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a serial PCM rejection");
}

struct Wire {
  explicit Wire(const SerialPcmFormat setup) : format(setup), receiver(setup) {}

  SerialPcmFormat format;
  SerialPcmReceiver receiver;
  std::uint64_t ticks = 0;
  std::vector<SerialPcmFrame> frames;

  void baseline(const bool active = false) {
    require(!receiver.observe(ticks,
                              {.clock = format.sample_edge == SerialPcmEdge::falling,
                               .frame_sync = active == format.frame_sync_high}),
            "A baseline is not a sampled edge");
  }

  void feed(const std::string_view bits, const std::string_view sync) {
    require(bits.size() == sync.size(), "Literal bit/sync expectations differ in size");
    for (std::size_t i = 0; i < bits.size(); ++i) {
      const bool active = sync[i] == '1';
      const auto data = bits[i] == '?' ? std::nullopt : std::optional{bits[i] == '1'};
      const bool high = format.sample_edge == SerialPcmEdge::rising;
      require(!receiver.observe(++ticks,
                                {.clock = !high,
                                 .frame_sync = active == format.frame_sync_high,
                                 .data = std::nullopt}),
              "No data is captured on the opposite edge");
      const auto frame = receiver.observe(
          ++ticks, {.clock = high, .frame_sync = active == format.frame_sync_high, .data = data});
      if (frame) { frames.push_back(*frame); }
    }
  }
};

void native_serial_pcm();
}
