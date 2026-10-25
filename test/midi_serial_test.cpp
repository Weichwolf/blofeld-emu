// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "midi/midi_serial.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace {
constexpr unsigned bus_hz = 30000000;
constexpr unsigned bit_cycles = 960;
constexpr unsigned byte_cycles = 9600;
constexpr unsigned fractional_bus_hz = 2000001;
constexpr unsigned baud = 31250;
constexpr unsigned frame_bits = 10;
constexpr std::size_t capacity = 4096;
constexpr std::uint8_t pattern = 0x55;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void rejects(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

void test_literal_wire_order_and_partial_bits() {
  blofeld::MidiSerialInput input(bus_hz);
  const auto idle = input.next_slice(byte_cycles);
  require(idle.level && idle.bus_cycles == byte_cycles,
          "Empty MIDI source supplies mark without inventing a frame");
  constexpr std::array<std::uint8_t, 1> bytes{0xa5};
  constexpr std::array<bool, frame_bits> levels{
      false, true, false, true, false, false, true, false, true, true};
  input.enqueue(bytes);
  for (const auto level : levels) {
    const auto slice = input.next_slice(byte_cycles);
    require(slice.level == level && slice.bus_cycles == bit_cycles && input.queued_bytes() == 1,
            "MIDI 8-N-1 has literal start, eight LSB-first A5 bits and stop; byte stays pending "
            "through stop");
    input.consume(bit_cycles - 1);
    const auto last = input.next_slice(byte_cycles);
    require(last.level == level && last.bus_cycles == 1,
            "Partial consumer slices must retain the exact last cycle of a bit");
    input.consume(1);
  }
  require(input.queued_bytes() == 0 && input.next_slice(1).level,
          "Only the complete stop bit releases its queued byte");
}

void test_rational_bit_boundaries_without_drift() {
  blofeld::MidiSerialInput input(fractional_bus_hz);
  constexpr std::size_t byte_count = 3125;
  std::array<std::uint8_t, byte_count> bytes{};
  bytes.fill(pattern);
  input.enqueue(bytes);
  std::uint64_t elapsed = 0;
  constexpr std::array<bool, frame_bits> levels{
      false, true, false, true, false, true, false, true, false, true};
  for (unsigned index = 0; index < baud; ++index) {
    const auto slice = input.next_slice(std::numeric_limits<unsigned>::max());
    require(slice.level == levels.at(index % frame_bits),
            "Back-to-back MIDI frames must not insert idle bits or reorder bytes");
    elapsed += slice.bus_cycles;
    const auto expected =
        ((static_cast<std::uint64_t>(index + 1) * fractional_bus_hz) + baud - 1) / baud;
    require(elapsed == expected,
            "Every bit boundary must equal independently rounded absolute MIDI time");
    input.consume(slice.bus_cycles);
  }
  require(elapsed == fractional_bus_hz && input.queued_bytes() == 0,
          "31250 MIDI bits must occupy exactly one declared bus-clock second without accumulated "
          "drift");
}

void test_capacity_wrap_and_rejected_consumption() {
  blofeld::MidiSerialInput input(bus_hz);
  std::array<std::uint8_t, capacity> bytes{};
  bytes.fill(pattern);
  input.enqueue(bytes);
  constexpr std::array<std::uint8_t, 1> extra{0xaa};
  rejects([&input, &extra] { input.enqueue(extra); },
          "Queue overflow must reject instead of dropping bytes");
  require(input.queued_bytes() == capacity, "Rejected enqueue leaves every pending byte intact");
  rejects([&input] { input.consume(bit_cycles + 1); },
          "Consumer must not cross an unoffered bit boundary");
  require(input.next_slice(byte_cycles).bus_cycles == bit_cycles && !input.next_slice(1).level,
          "Rejected clock consumption cannot change the first start bit");
  const auto drain_byte = [&input](const std::array<bool, frame_bits> &levels) {
    for (const auto level : levels) {
      require(input.next_slice(byte_cycles).level == level,
              "Ring wrap/backpressure must preserve literal wire bytes");
      input.consume(bit_cycles);
    }
  };
  constexpr std::array<bool, frame_bits> original{
      false, true, false, true, false, true, false, true, false, true};
  constexpr std::array<bool, frame_bits> appended{
      false, false, true, false, true, false, true, false, true, true};
  constexpr std::size_t drained = 17;
  for (std::size_t index = 0; index < drained; ++index) { drain_byte(original); }
  for (std::size_t index = 0; index < drained; ++index) { input.enqueue(extra); }
  for (std::size_t index = drained; index < capacity; ++index) { drain_byte(original); }
  for (std::size_t index = 0; index < drained; ++index) { drain_byte(appended); }
  require(input.queued_bytes() == 0, "Ring wrap must drain all original and appended bytes once");
}

void test_chunking_retains_clock_epoch_until_idle() {
  blofeld::MidiSerialInput input(fractional_bus_hz);
  constexpr std::array<std::uint8_t, 1> byte{pattern};
  constexpr unsigned byte_count = 3125;
  std::uint64_t elapsed = 0;
  for (unsigned index = 0; index < byte_count; ++index) {
    input.enqueue(byte);
    for (unsigned bit = 0; bit < frame_bits; ++bit) {
      const auto slice = input.next_slice(byte_cycles);
      input.consume(slice.bus_cycles);
      elapsed += slice.bus_cycles;
    }
    const auto expected =
        ((static_cast<std::uint64_t>(index + 1) * frame_bits * fractional_bus_hz) + baud - 1) /
        baud;
    require(elapsed == expected && input.queued_bytes() == 0,
            "Refilling an empty queue without elapsed idle time must not accumulate one rounding "
            "error per byte");
  }
  require(elapsed == fractional_bus_hz,
          "Single-byte enqueue chunks must retain the same exact one-second duration");
  input.consume(1);
  input.enqueue(byte);
  unsigned fresh_duration = 0;
  for (unsigned bit = 0; bit < frame_bits; ++bit) {
    const auto slice = input.next_slice(byte_cycles);
    fresh_duration += slice.bus_cycles;
    input.consume(slice.bus_cycles);
  }
  constexpr unsigned expected_fresh_duration = 641;
  require(fresh_duration == expected_fresh_duration,
          "Positive idle time must begin a new asynchronous source epoch");
  input.consume(1);
  input.enqueue(byte);
  constexpr unsigned fresh_bit_cycles = 65;
  require(input.next_slice(byte_cycles).bus_cycles == fresh_bit_cycles,
          "Idle after a rounded frame must discard its residual source phase");
}

void test_snapshot_fragmentation_and_clock_bounds() {
  rejects([] { static_cast<void>(blofeld::MidiSerialInput(0)); }, "Unknown clock must reject");
  rejects([] { static_cast<void>(blofeld::MidiSerialInput(baud - 1)); },
          "Sub-bit clock resolution must reject");
  blofeld::MidiSerialInput input(fractional_bus_hz);
  constexpr std::array<std::uint8_t, 2> bytes{0xf8, 0x90};
  input.enqueue(bytes);
  constexpr unsigned fragment = 7;
  input.consume(fragment);
  auto restored = input;
  std::uint64_t elapsed = fragment;
  while (input.queued_bytes() != 0) {
    const auto slice = input.next_slice(fragment);
    const auto copied = restored.next_slice(fragment);
    require(slice.level == copied.level && slice.bus_cycles == copied.bus_cycles,
            "A copy must retain both queue and sub-bit fractional phase");
    input.consume(slice.bus_cycles);
    restored.consume(copied.bus_cycles);
    elapsed += slice.bus_cycles;
  }
  const auto expected =
      ((std::uint64_t{frame_bits} * bytes.size() * fractional_bus_hz) + baud - 1) / baud;
  require(elapsed == expected && restored.queued_bytes() == 0,
          "Fragmented replay must retain independent complete-frame duration");
  blofeld::MidiSerialInput maximum(std::numeric_limits<std::uint32_t>::max());
  maximum.enqueue(bytes);
  constexpr unsigned maximum_bit_cycles = 137439;
  require(maximum.next_slice(std::numeric_limits<unsigned>::max()).bus_cycles == maximum_bit_cycles,
          "Maximum input clock must use a wide rational intermediate");
  maximum.consume(maximum_bit_cycles);
  maximum.consume(0);
  require(maximum.queued_bytes() == bytes.size() && maximum.next_slice(0).bus_cycles == 0,
          "Zero clock slices cannot consume a byte or overflow its phase");
}
}

int main() {
  try {
    test_literal_wire_order_and_partial_bits();
    test_rational_bit_boundaries_without_drift();
    test_capacity_wrap_and_rejected_consumption();
    test_chunking_retains_clock_epoch_until_idle();
    test_snapshot_fragmentation_and_clock_bounds();
    std::cout
        << "MIDI serial literal frames, fractional clocks, bounded queue and replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
