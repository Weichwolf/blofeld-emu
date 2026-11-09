// SPDX-License-Identifier: GPL-3.0-only
#include "io/midi_port.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::MidiPort;
constexpr std::uint8_t note_on = 0x90, note = 60, velocity = 127;
constexpr std::array message{note_on, note, velocity};

void require(const bool condition, const char *message_text) {
  if (!condition) { throw std::runtime_error(message_text); }
}

void ordered_packets() {
  MidiPort port;
  constexpr std::uint64_t first = 100;
  require(!port.peek_input() && !port.read_output() && port.time() == 0 &&
              port.write_input({}, first) && port.write_input(message, first) &&
              port.write_input(message, first),
          "Empty ports manufacture no bytes and packets can share a musical timestamp");
  for (unsigned index = 0; index < message.size() * 2; ++index) {
    const MidiPort::Event wanted{.ticks = first + (index * MidiPort::byte_ticks),
                                 .byte = message[index % message.size()]};
    require(port.peek_input() == wanted && port.peek_input() == wanted,
            "Packet bytes retain order and DIN spacing; lookahead does not consume them");
    port.pop_input();
    require(port.write_output(wanted) && port.read_output() == wanted && !port.read_output(),
            "Actual output is transferred exactly once with its completion timestamp");
  }
  require(!port.peek_input() && port.error() == nullptr, "Packets end without invented padding");
  port.publish_time(first);
  require(port.time() == first, "The host sees published device time rather than host time");
}

void wrap_and_overload() {
  MidiPort port;
  std::array<std::uint8_t, MidiPort::capacity> bytes{};
  constexpr auto byte_mask = std::numeric_limits<std::uint8_t>::max();
  for (unsigned index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(index & byte_mask);
  }
  require(port.write_input(bytes, 0), "Fill the entire bounded input");
  for (unsigned index = 0; index < bytes.size() / 2; ++index) { port.pop_input(); }
  require(port.write_input(std::span(bytes).first(bytes.size() / 2), 0),
          "Released slots can be reused across the ring boundary");
  for (unsigned index = 0; index < bytes.size(); ++index) {
    const auto sequence = index + (bytes.size() / 2);
    require(port.peek_input() ==
                MidiPort::Event{.ticks = sequence * MidiPort::byte_ticks,
                                .byte = static_cast<std::uint8_t>(sequence & byte_mask)},
            "Wrapped input retains exact data and timestamps");
    port.pop_input();
  }
  require(!port.peek_input(), "All wrapped input was consumed");
  MidiPort full;
  require(full.write_input(bytes, 0) && !full.write_input(message, 0) && full.error() != nullptr &&
              full.peek_input() == MidiPort::Event{},
          "Input overflow is visible and cannot overwrite previously accepted data");
  MidiPort output;
  for (unsigned index = 0; index < MidiPort::capacity; ++index) {
    require(output.write_output({.ticks = index, .byte = note_on}), "Fill bounded output");
  }
  require(!output.write_output({.ticks = MidiPort::capacity, .byte = note}) &&
              output.error() != nullptr &&
              output.read_output() == MidiPort::Event{.ticks = 0, .byte = note_on},
          "Output overflow preserves the oldest real firmware byte");
}

void invalid_times() {
  MidiPort backwards;
  require(backwards.write_input(message, 1) && !backwards.write_input(message, 0) &&
              backwards.error() != nullptr,
          "Out-of-order host timestamps fail explicitly");
  MidiPort overflow;
  require(!overflow.write_input(message, std::numeric_limits<std::uint64_t>::max()) &&
              !overflow.peek_input() && overflow.error() != nullptr,
          "An unrepresentable character interval never partially enters the queue");
  MidiPort output;
  require(output.write_output({.ticks = 1, .byte = note_on}) &&
              !output.write_output({.ticks = 0, .byte = note}) && output.error() != nullptr,
          "Firmware output cannot reverse the common device timeline");
}

void concurrent_exchange() {
  MidiPort port;
  constexpr unsigned rounds = 1024;
  constexpr unsigned bytes = rounds * message.size();
  bool coherent = true;
  std::jthread mcu([&](const std::stop_token &stop) {
    for (unsigned index = 0; index < bytes; ++index) {
      while (!port.peek_input()) {
        if (stop.stop_requested()) { return; }
        std::this_thread::yield();
      }
      const auto event = *port.peek_input();
      coherent = coherent && event == MidiPort::Event{.ticks = index * MidiPort::byte_ticks,
                                                      .byte = message[index % message.size()]};
      port.pop_input();
      const bool posted = port.write_output(event);
      coherent = coherent && posted;
      if (!posted) { return; }
      port.publish_time(event.ticks);
    }
  });
  for (unsigned round = 0; round < rounds; ++round) {
    require(port.write_input(message, 0), "One host owner submits ordered packets");
    for (unsigned index = 0; index < message.size(); ++index) {
      std::optional<MidiPort::Event> event;
      while (!(event = port.read_output())) {
        require(port.error() == nullptr, "Concurrent output cannot silently stop on overload");
        std::this_thread::yield();
      }
      require(event == MidiPort::Event{.ticks = ((round * message.size()) + index) *
                                                MidiPort::byte_ticks,
                                       .byte = message[index]},
              "The host observes coherent output slots through concurrent ring reuse");
    }
  }
  mcu.join();
  require(coherent && !port.peek_input() && !port.read_output() && port.error() == nullptr &&
              port.time() == (bytes - 1) * MidiPort::byte_ticks,
          "Both owners finish with exact data, device time and empty queues");
}
}

int main() try {
  ordered_packets();
  wrap_and_overload();
  invalid_times();
  concurrent_exchange();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
