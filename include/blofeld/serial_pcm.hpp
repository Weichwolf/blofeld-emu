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
};

struct SerialPcmPins {
  std::optional<bool> clock = std::nullopt;
  std::optional<bool> frame_sync = std::nullopt;
  std::optional<bool> data = std::nullopt;
};

struct SerialPcmFrame {
  static constexpr unsigned maximum_channels = 32;
  std::array<std::int32_t, maximum_channels> samples{};
  unsigned channels = 0;
  std::uint64_t first_tick = 0;
  std::uint64_t last_tick = 0;
  bool operator==(const SerialPcmFrame &) const = default;
};

// Bounded receiver; copies retain alignment, partial words and the clock phase.
// Success allocates nothing. Fault diagnostics throw and are not RT callbacks.
class SerialPcmReceiver {
public:
  explicit SerialPcmReceiver(SerialPcmFormat format);
  [[nodiscard]] std::optional<SerialPcmFrame> observe(std::uint64_t ticks, SerialPcmPins pins);
  void reset();

  [[nodiscard]] std::uint64_t completed_frames() const { return frames_; }

  [[nodiscard]] bool locked() const { return locked_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  [[noreturn]] void fail(std::uint64_t ticks, std::string_view reason);
  [[nodiscard]] std::optional<SerialPcmFrame>
  sample(std::uint64_t ticks, bool active, std::optional<bool> data);
  [[nodiscard]] std::int32_t signed_word() const;
  SerialPcmFormat format_;
  SerialPcmFrame frame_;
  std::optional<bool> previous_clock_;
  std::optional<std::uint64_t> previous_tick_;
  std::uint64_t frames_ = 0;
  std::int64_t word_modulus_ = 0;
  std::uint32_t word_sign_ = 0;
  std::uint32_t word_ = 0;
  unsigned slot_ = 0;
  unsigned bit_ = 0;
  bool seen_inactive_ = false;
  bool locked_ = false;
  bool faulted_ = false;
};
}
