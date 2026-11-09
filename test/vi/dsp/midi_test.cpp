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
  for (unsigned bit = 0; bit < levels.size(); ++bit) {
    const auto now = start + (bit * bit_ticks);
    require(input.advance(now) && input.level() == levels[bit] &&
                input.deadline() == now + bit_ticks && input.advance(now) &&
                input.level() == levels[bit],
            "Start, eight LSB-first data bits and stop have independent 32-us boundaries");
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
  input.reset(start + (3 * bit_ticks) + 1);
  require(!input.level() && input.deadline() == start + (4 * bit_ticks),
          "Reset in a character samples its actual level without inventing a fresh start");
  constexpr unsigned stop = 9;
  require(input.advance(start + (stop * bit_ticks)) && input.level(), "The stop bit is high");
  const auto second = start + MidiPort::byte_ticks;
  require(input.advance(second) && !input.level() && input.deadline() == second + bit_ticks &&
              port.peek_input(receiver) == MidiPort::Event{.ticks = second, .byte = bytes[1]},
          "A back-to-back character starts at the prior stop's end without an invented gap");
  input.reset(second + (2 * bit_ticks));
  require(input.level() && input.deadline() == second + (3 * bit_ticks),
          "A reset during a high data bit retains its physical phase");
  input.reset(second + MidiPort::byte_ticks);
  require(input.level() && !input.deadline() && !port.peek_input(receiver) && port.peek_input(),
          "Bytes that elapsed during DSP reset are not replayed after its release");
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
  missing_boundaries_and_invalid_time();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
