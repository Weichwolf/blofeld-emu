// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <sound/asequencer.h>

namespace blofeld::detail {
struct SequenceBytes {
  std::array<std::uint8_t, 12> data{};
  std::size_t size = 0;
};

// Fixed MIDI-1 events only. SysEx payloads remain an ordered byte stream.
[[nodiscard]] SequenceBytes sequence_bytes(const snd_seq_event &event);

struct SequencePacket {
  snd_seq_event event{};
  std::array<std::uint8_t, 256> data{};
  std::size_t size = 0;
};

// Bounded wire parser. No timing, device replies, subscriptions or synthesis.
class SequenceEncoder {
public:
  explicit SequenceEncoder(std::size_t fragment_limit = 256);
  // False means an earlier SysEx fragment became ready; retry this same byte
  // only after delivering that packet and calling consume().
  bool feed(std::uint8_t value);
  [[nodiscard]] const SequencePacket *pending() const;
  void consume();

private:
  void fixed();
  void fragment();
  void status(std::uint8_t value);
  void realtime(std::uint8_t value);
  SequencePacket packet_;
  std::array<std::uint8_t, 3> message_{};
  std::array<std::uint8_t, 256> sysex_bytes_{};
  std::size_t sysex_size_ = 0;
  std::size_t fragment_limit_ = 256;
  unsigned used_ = 0, expected_ = 0;
  std::uint8_t running_ = 0;
  bool sysex_ = false, ready_ = false;
};
}
