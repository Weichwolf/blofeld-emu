// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/midi.hpp"
#include "io/midi_port.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::MidiPort;
using blofeld::vi::dsp::MidiInput;
constexpr auto receiver = MidiPort::Receiver::dsp;
constexpr std::uint64_t bit_ticks = 1920, start = 100;
constexpr std::uint8_t datum = 0xa5;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void framing_and_ownership() {
  MidiPort port;
  MidiInput input(&port);
  require(port.write_input(std::array{datum}, start), "Admit one real DIN character");
  input.reset(0);
  require(input.level() && input.deadline() == start && input.advance(start - 1) && input.level(),
          "The receiver stays idle high until the scheduled character start");
  constexpr std::array levels{false, true, false, true, false, false, true, false, true, true};
  constexpr std::array next{1U, 2U, 3U, 4U, 6U, 6U, 7U, 8U, 10U, 10U};
  for (unsigned bit = 0; bit < levels.size(); ++bit) {
    const auto now = start + (bit * bit_ticks);
    require(input.advance(now) && input.level() == levels[bit] &&
                input.deadline() == start + (next[bit] * bit_ticks) && input.advance(now) &&
                input.level() == levels[bit],
            "All ten bit levels remain observable; only the next change or byte end wakes the DSP");
  }
  require(input.advance(start + MidiPort::byte_ticks) && input.level() && !input.deadline() &&
              !port.peek_input(receiver) &&
              port.peek_input() == MidiPort::Event{.ticks = start, .byte = datum},
          "Finishing DSP reception retires only its own cursor, preserving the MCU observer");
}

void adjacent_characters_and_reset() {
  MidiPort port;
  MidiInput input(&port);
  constexpr std::array<std::uint8_t, 2> bytes{0, 0xff};
  require(port.write_input(bytes, start), "Admit adjacent DIN characters");
  constexpr unsigned stop = 9;
  input.reset(start + (3 * bit_ticks) + 1);
  require(!input.level() && input.deadline() == start + (stop * bit_ticks),
          "Reset in a character samples its actual level without inventing a fresh start");
  require(input.advance(start + (stop * bit_ticks)) && input.level(), "The stop bit is high");
  const auto second = start + MidiPort::byte_ticks;
  require(input.advance(second) && !input.level() && input.deadline() == second + bit_ticks &&
              port.peek_input(receiver) == MidiPort::Event{.ticks = second, .byte = bytes[1]},
          "A back-to-back character starts at the prior stop's end without an invented gap");
  input.reset(second + (2 * bit_ticks));
  require(input.level() && input.deadline() == second + MidiPort::byte_ticks,
          "A reset during a high data bit retains its physical phase");
  input.reset(second + MidiPort::byte_ticks);
  require(input.level() && !input.deadline() && !port.peek_input(receiver) && port.peek_input(),
          "Bytes that elapsed during DSP reset are not replayed after its release");
}

void every_character() {
  constexpr unsigned characters = 256;
  constexpr unsigned data_bits = 8;
  constexpr unsigned stop = data_bits + 1;
  constexpr unsigned end = stop + 1;
  for (unsigned value = 0; value < characters; ++value) {
    MidiPort port;
    MidiInput input(&port);
    require(port.write_input(std::array{static_cast<std::uint8_t>(value)}, start),
            "Admit every DIN byte value");
    input.reset(0);
    unsigned transition = 0;
    const auto level = [value](const unsigned bit) {
      if (bit == 0) { return false; }
      if (bit >= stop) { return true; }
      return ((value >> (bit - 1)) & 1U) != 0;
    };
    for (unsigned bit = 0; bit < end; ++bit) {
      const auto now = start + (bit * bit_ticks);
      if (bit == transition) {
        require(input.deadline() == now && input.advance(now),
                "Advance solely at the scheduled line transitions");
        transition = bit + 1;
        while (transition < end && level(transition) == level(bit)) { ++transition; }
        require(input.deadline() == start + (transition * bit_ticks),
                "The next wake preserves the first changed level or byte end");
      }
      require(input.level() == level(bit), "Every bit remains visible between DSP wakes");
    }
    require(input.deadline() == start + MidiPort::byte_ticks &&
                input.advance(start + MidiPort::byte_ticks) && !input.deadline(),
            "The receiver must retire every character");
  }
}

void missing_boundaries_and_invalid_time() {
  MidiPort port;
  MidiInput input(&port);
  require(port.write_input(std::array{datum}, start), "Admit the missed-boundary witness");
  input.reset(0);
  require(!input.advance(start + MidiPort::byte_ticks) && input.error() != nullptr &&
              port.peek_input(receiver),
          "Execution cannot skip an entire unseen character or silently claim its consumption");
  input.reset(start);
  require(!input.advance(start - 1) && input.error() != nullptr,
          "An executing DSP input cannot move device time backwards");
  MidiInput disconnected;
  require(!disconnected.connected() && disconnected.level() && !disconnected.deadline() &&
              disconnected.advance(start),
          "An absent host binding creates no queued MIDI data or deadlines");
}
}

int main() try {
  framing_and_ownership();
  adjacent_characters_and_reset();
  every_character();
  missing_boundaries_and_invalid_time();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
