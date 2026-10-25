// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/midi_serial.hpp"
#include "midi/midi_timed.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
constexpr std::uint64_t fractional_hz = 31250003;
constexpr std::uint64_t start = 123;
constexpr unsigned all_bytes = 256;
constexpr unsigned expected_baud = 31250;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected rejected timed MIDI operation");
}

void test_all_bytes_and_independent_rational_grid() {
  blofeld::MidiTimedInput input({.timebase_hz = fractional_hz, .start_ticks = start});
  std::array<std::uint8_t, all_bytes> bytes{};
  for (unsigned byte = 0; byte < bytes.size(); ++byte) {
    bytes.at(byte) = static_cast<std::uint8_t>(byte);
  }
  input.enqueue(bytes);
  constexpr unsigned data_bits = 8;
  constexpr unsigned frame_bits = data_bits + 2;
  constexpr unsigned total_frames = 8192;
  constexpr std::uint64_t chunk_bits = std::uint64_t{all_bytes} * frame_bits;
  constexpr std::uint64_t total_bits = std::uint64_t{total_frames} * frame_bits;
  for (std::uint64_t index = 0; index <= total_bits; ++index) {
    if (index != 0 && index < total_bits && index % chunk_bits == 0) { input.enqueue(bytes); }
    const auto event = input.next_event();
    const auto expected = start + ((index * fractional_hz + expected_baud - 1) / expected_baud);
    require(event.ticks == expected && input.next_event() == event,
            "Every deadline matches independent absolute 31250-baud ceiling arithmetic and pure "
            "queries");
    if (index == total_bits) {
      require(event.level && !event.frame_bit,
              "The completed last stop bit leads to explicit idle");
    } else {
      const auto bit = static_cast<unsigned>(index % frame_bits);
      const auto value = static_cast<unsigned>((index / frame_bits) % all_bytes);
      const bool level = bit == frame_bits - 1 || (bit != 0 && ((value >> (bit - 1)) & 1U) != 0);
      require(event.frame_bit == bit && event.level == level,
              "Every byte independently supplies start-low, LSB-first data and stop-high");
    }
    input.consume(event.ticks);
  }
  require(input.queued_bytes() == 0 && input.completed_frames() == total_frames,
          "All 8192 frames cross ring wraps once, retaining each active byte through its stop "
          "interval");
}

void test_idle_enqueue_and_partial_copy() {
  blofeld::MidiTimedInput input({.timebase_hz = fractional_hz, .start_ticks = start});
  for (unsigned idle = 0; idle < 7; ++idle) {
    const auto event = input.next_event();
    require(event.level && !event.frame_bit, "Empty queues keep a stable explicit idle grid");
    input.consume(event.ticks);
  }
  const auto pinned = input.next_event().ticks;
  const std::array<std::uint8_t, 1> byte{0xa5};
  input.enqueue(byte);
  require(input.next_event().ticks == pinned, "Enqueue cannot change a bound scheduler deadline");
  constexpr std::array<bool, 10> levels{
      false, true, false, true, false, false, true, false, true, true};
  for (unsigned bit = 0; bit < 4; ++bit) {
    require(input.next_event().level == levels.at(bit), "Literal A5 prefix bits");
    input.consume(input.next_event().ticks);
  }
  const auto saved = input;
  for (unsigned replay = 0; replay < 2; ++replay) {
    input = saved;
    for (unsigned bit = 4; bit < levels.size(); ++bit) {
      require(input.next_event().level == levels.at(bit) && input.next_event().frame_bit == bit,
              "A copy preserves the exact next partial-frame bit");
      input.consume(input.next_event().ticks);
    }
    input.consume(input.next_event().ticks);
    require(input.queued_bytes() == 0 && input.completed_frames() == 1,
            "Each copied suffix removes the active byte exactly once after its stop bit");
  }
}

void test_declared_reference_gap() {
  constexpr std::uint64_t period = 292410;
  blofeld::MidiTimedInput input(
      {.timebase_hz = fractional_hz, .reference_bit_ticks = period, .reference_idle_bits = 1});
  const std::array<std::uint8_t, 1> byte{0};
  input.enqueue(byte);
  constexpr unsigned intervals = 11;
  for (unsigned bit = 0; bit <= intervals; ++bit) {
    const auto event = input.next_event();
    require(event.ticks == bit * period && event.level == (bit >= 9),
            "An explicit reference period/gap retains all start/data/stop/idle intervals");
    input.consume(event.ticks);
  }
  require(input.completed_frames() == 1 && input.queued_bytes() == 0,
          "Reference gap precedes completion");
}

void test_bounds_and_failed_commit() {
  blofeld::MidiTimedInput input({.timebase_hz = fractional_hz});
  std::array<std::uint8_t, blofeld::MidiSerialInput::capacity> full{};
  const std::array<std::uint8_t, 1> extra{0x90};
  input.enqueue(full);
  const auto before = input.next_event();
  static_cast<void>(rejects([&] { input.enqueue(extra); }));
  static_cast<void>(rejects([&] { input.consume(before.ticks + 1); }));
  require(input.next_event() == before && input.queued_bytes() == full.size() &&
              input.observed_events() == 0,
          "Overflow/wrong-deadline failures retain the exact source prefix");
  blofeld::MidiTimedInput exhausted({.timebase_hz = fractional_hz,
                                     .start_ticks = std::numeric_limits<std::uint64_t>::max() - 1,
                                     .reference_bit_ticks = 2});
  const auto final = exhausted.next_event();
  static_cast<void>(rejects([&] { exhausted.consume(final.ticks); }));
  require(exhausted.next_event() == final && exhausted.observed_events() == 0,
          "Timestamp exhaustion cannot wrap or partially commit an input");
}
}

int main() {
  try {
    test_all_bytes_and_independent_rational_grid();
    test_idle_enqueue_and_partial_copy();
    test_declared_reference_gap();
    test_bounds_and_failed_commit();
    std::cout
        << "Timed MIDI exact-byte/rational-grid, idle, partial-copy and strict bounds passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
