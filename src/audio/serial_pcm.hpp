// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
enum class SerialPcmEdge : std::uint8_t { unqualified, rising, falling };

// One MSB-first, left-justified data wire; word-length FS marks the first slot.
// This explicit receiver contract does not select an ESAI pin or physical rate.
struct SerialPcmFormat {
  static constexpr unsigned dsp_word_bits = 24;
  unsigned slot_bits = 32;
  unsigned word_bits = dsp_word_bits;
  unsigned channels = 2;
  SerialPcmEdge sample_edge = SerialPcmEdge::rising;
  bool frame_sync_high = true;
  bool operator==(const SerialPcmFormat &) const = default;
};

struct SerialPcmPins {
  std::optional<bool> clock = std::nullopt;
  std::optional<bool> frame_sync = std::nullopt;
  std::optional<bool> data = std::nullopt;
  bool operator==(const SerialPcmPins &) const = default;
};

// Order is a caller-declared causal ordinal within the same absolute tick.
struct SerialPcmStamp {
  std::uint64_t ticks = 0;
  std::uint64_t order = 0;
  bool operator==(const SerialPcmStamp &) const = default;
};

struct SerialPcmFrame {
  static constexpr unsigned maximum_channels = 32;
  std::array<std::int32_t, maximum_channels> samples{};
  unsigned channels = 0;
  std::uint64_t first_tick = 0;
  std::uint64_t last_tick = 0;
  std::uint64_t first_order = 0;
  std::uint64_t last_order = 0;
  std::optional<std::uint64_t> bit_ticks = std::nullopt;
  bool uniform_clock = true;
  bool operator==(const SerialPcmFrame &) const = default;
};

void validate_pcm_format(SerialPcmFormat format);
}
