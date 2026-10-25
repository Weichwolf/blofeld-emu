// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "midi/midi_serial.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld {
struct MidiWireTiming {
  std::uint64_t timebase_hz = 0;
  std::uint64_t start_ticks = 0;
  std::optional<std::uint64_t> reference_bit_ticks = std::nullopt;
  unsigned reference_idle_bits = 0;
};

struct MidiTimedEvent {
  std::uint64_t ticks = 0;
  bool level = true;
  std::optional<unsigned> frame_bit = std::nullopt;
  bool operator==(const MidiTimedEvent &) const = default;
};

// Fixed free-running bit grid, including empty-queue idle observations. Enqueue
// never changes a scheduler's pinned next deadline. Default is rational 31250 baud.
class MidiTimedInput {
public:
  explicit MidiTimedInput(MidiWireTiming timing);
  void enqueue(std::span<const std::uint8_t> bytes);
  [[nodiscard]] MidiTimedEvent next_event() const;
  // Commit after the peripheral accepts the observed level, at exactly its deadline.
  void consume(std::uint64_t at);

  [[nodiscard]] std::size_t queued_bytes() const { return size_; }

  [[nodiscard]] std::uint64_t completed_frames() const { return completed_; }

  [[nodiscard]] std::uint64_t observed_events() const { return events_; }

private:
  struct Position {
    std::size_t head = 0;
    std::size_t size = 0;
    unsigned bit = 0;
    bool completed = false;
  };

  [[nodiscard]] Position position() const;
  std::array<std::uint8_t, MidiSerialInput::capacity> queue_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
  unsigned bit_ = 0;
  unsigned frame_bits_ = MidiSerialInput::frame_bits;
  std::uint64_t next_ = 0;
  std::uint64_t quotient_ = 0;
  unsigned remainder_ = 0;
  unsigned phase_ = 0;
  std::uint64_t completed_ = 0;
  std::uint64_t events_ = 0;
};
}
