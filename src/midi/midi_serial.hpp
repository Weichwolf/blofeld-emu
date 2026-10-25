// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld {

struct MidiSerialSlice {
  unsigned bus_cycles = 0;
  bool level = true;
};

// Ideal MIDI 1.0 8-N-1 source, expressed in an explicitly supplied bus clock.
// Copies retain queued bytes, the partial frame and rational bit-clock phase.
class MidiSerialInput {
public:
  static constexpr unsigned baud = 31250;
  static constexpr unsigned frame_bits = 10;
  static constexpr std::size_t capacity = 4096;
  explicit MidiSerialInput(std::uint32_t bus_hz);
  // All-or-nothing bounded enqueue. No message parsing or byte substitution.
  void enqueue(std::span<const std::uint8_t> bytes);
  [[nodiscard]] MidiSerialSlice next_slice(unsigned maximum_cycles) const;
  // Commit at most one offered bit interval, after its consumer succeeds.
  void consume(unsigned cycles);

  [[nodiscard]] std::size_t queued_bytes() const { return size_; }

private:
  [[nodiscard]] unsigned until_bit() const;
  [[nodiscard]] bool next_level() const;
  std::uint32_t bus_hz_;
  std::uint32_t phase_ = 0;
  std::array<std::uint8_t, capacity> queue_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
  unsigned bit_ = 0;
};

}
