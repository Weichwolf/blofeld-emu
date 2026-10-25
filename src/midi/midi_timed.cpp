// SPDX-License-Identifier: GPL-3.0-only
#include "midi/midi_timed.hpp"

#include "common/bus.hpp"
#include "midi/midi_serial.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace blofeld {
MidiTimedInput::MidiTimedInput(const MidiWireTiming timing) : next_(timing.start_ticks) {
  constexpr unsigned maximum_idle_bits = 16;
  if (timing.timebase_hz == 0 || timing.reference_idle_bits > maximum_idle_bits ||
      (!timing.reference_bit_ticks && timing.reference_idle_bits != 0)) {
    throw Fault("Invalid timed MIDI wire/reference framing contract");
  }
  if (timing.reference_bit_ticks) {
    if (*timing.reference_bit_ticks == 0) {
      throw Fault("Reference MIDI bit period must be positive");
    }
    quotient_ = *timing.reference_bit_ticks;
  } else {
    if (timing.timebase_hz < MidiSerialInput::baud) {
      throw Fault("Timed MIDI requires at least one common tick per wire bit");
    }
    quotient_ = timing.timebase_hz / MidiSerialInput::baud;
    remainder_ = static_cast<unsigned>(timing.timebase_hz % MidiSerialInput::baud);
  }
  frame_bits_ += timing.reference_idle_bits;
}

void MidiTimedInput::enqueue(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() > queue_.size() - size_) { throw Fault("Timed MIDI queue capacity exceeded"); }
  for (const auto byte : bytes) {
    queue_.at((head_ + size_) % queue_.size()) = byte;
    ++size_;
  }
}

MidiTimedInput::Position MidiTimedInput::position() const {
  if (bit_ != frame_bits_) { return {.head = head_, .size = size_, .bit = bit_}; }
  return {.head = (head_ + 1) % queue_.size(), .size = size_ - 1, .completed = true};
}

MidiTimedEvent MidiTimedInput::next_event() const {
  const auto next = position();
  if (next.size == 0) { return {.ticks = next_}; }
  constexpr unsigned stop_bit = MidiSerialInput::frame_bits - 1;
  const bool level = next.bit >= stop_bit ||
                     (next.bit != 0 && ((queue_.at(next.head) >> (next.bit - 1)) & 1U) != 0);
  return {.ticks = next_, .level = level, .frame_bit = next.bit};
}

void MidiTimedInput::consume(const std::uint64_t at) {
  if (at != next_) { throw Fault("Timed MIDI observation does not match its pinned deadline"); }
  const auto next = position();
  const unsigned sum = phase_ + remainder_;
  const auto rounded = (sum + MidiSerialInput::baud - 1U) / MidiSerialInput::baud;
  const auto duration = quotient_ + rounded - (phase_ != 0 ? 1U : 0U);
  if (duration > std::numeric_limits<std::uint64_t>::max() - next_ ||
      events_ == std::numeric_limits<std::uint64_t>::max() ||
      (next.completed && completed_ == std::numeric_limits<std::uint64_t>::max())) {
    throw Fault("Timed MIDI wire counters exhausted");
  }
  next_ += duration;
  phase_ = sum % MidiSerialInput::baud;
  head_ = next.head;
  size_ = next.size;
  bit_ = next.size != 0 ? next.bit + 1 : 0;
  if (next.completed) { ++completed_; }
  ++events_;
}
}
