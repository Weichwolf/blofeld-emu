// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/midi_serial.hpp"

#include "blofeld/bus.hpp"

#include <algorithm>
#include <cstdint>
#include <span>

namespace blofeld {

MidiSerialInput::MidiSerialInput(const std::uint32_t bus_hz) : bus_hz_(bus_hz) {
  if (bus_hz < baud) { throw Fault("MIDI serial source requires at least one bus clock per bit"); }
}

void MidiSerialInput::enqueue(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() > capacity - size_) { throw Fault("MIDI serial input queue capacity exceeded"); }
  for (const auto byte : bytes) {
    queue_.at((head_ + size_) % capacity) = byte;
    ++size_;
  }
}

unsigned MidiSerialInput::until_bit() const {
  const auto remaining = static_cast<std::uint64_t>(bus_hz_) - phase_;
  return static_cast<unsigned>((remaining + baud - 1) / baud);
}

bool MidiSerialInput::next_level() const {
  if (size_ == 0 || bit_ == frame_bits - 1) { return true; }
  if (bit_ == 0) { return false; }
  return ((queue_.at(head_) >> (bit_ - 1)) & 1U) != 0;
}

MidiSerialSlice MidiSerialInput::next_slice(const unsigned maximum_cycles) const {
  return {.bus_cycles = size_ == 0 ? maximum_cycles : std::min(maximum_cycles, until_bit()),
          .level = next_level()};
}

void MidiSerialInput::consume(const unsigned cycles) {
  if (cycles == 0) { return; }
  if (size_ == 0) {
    phase_ = 0;
    return;
  }
  if (cycles > until_bit()) { throw Fault("MIDI serial consumer crossed an offered bit boundary"); }
  const auto next_phase =
      static_cast<std::uint64_t>(phase_) + (static_cast<std::uint64_t>(cycles) * baud);
  if (next_phase < bus_hz_) {
    phase_ = static_cast<std::uint32_t>(next_phase);
    return;
  }
  phase_ = static_cast<std::uint32_t>(next_phase - bus_hz_);
  if (++bit_ != frame_bits) { return; }
  bit_ = 0;
  head_ = (head_ + 1) % capacity;
  --size_;
}

}
