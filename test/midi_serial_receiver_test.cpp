// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_sci.hpp"
#include "midi/midi_serial_receiver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string_view>

namespace {
using namespace blofeld;
constexpr std::uint64_t hz = 1000000;
constexpr unsigned start = 32;
constexpr std::uint64_t bit = 32;
constexpr unsigned start_center = 48;
constexpr unsigned data_zero_center = 80;
constexpr unsigned data_one_edge = 96;
constexpr unsigned stop = 320;
constexpr unsigned completed = 336;
constexpr unsigned next_start = 352;
constexpr unsigned next_start_center = 368;
constexpr unsigned next_data_zero_edge = 384;
constexpr unsigned next_completed = 656;
constexpr unsigned fractional_start_center = 49;
constexpr unsigned fractional_stop = 321;
constexpr unsigned fractional_completed = 337;
constexpr unsigned early_start_glitch = 40;
constexpr unsigned idle_recovery = 20;
constexpr unsigned before_start = 31;
constexpr std::uint8_t all_high = 0xff;

void require(const bool condition, const char *reason) {
  if (!condition) { throw Fault(reason); }
}

template <class Function> void rejects(Function function, const std::string_view text) {
  bool rejected = false;
  try {
    function();
  } catch (const Fault &error) { rejected = std::string_view(error.what()).contains(text); }
  require(rejected, "Expected a visible serial decoder refusal");
}

struct Edge {
  std::uint64_t ticks;
  bool high;
};

constexpr std::array alternating{Edge{.ticks = 32, .high = false},
                                 Edge{.ticks = 64, .high = true},
                                 Edge{.ticks = 96, .high = false},
                                 Edge{.ticks = 128, .high = true},
                                 Edge{.ticks = 160, .high = false},
                                 Edge{.ticks = 192, .high = true},
                                 Edge{.ticks = 224, .high = false},
                                 Edge{.ticks = 256, .high = true},
                                 Edge{.ticks = 288, .high = false},
                                 Edge{.ticks = 320, .high = true}};

void literals() {
  MidiSerialReceiver zero(hz);
  require(!zero.observe(0, true) && !zero.observe(start, false), "Start does not invent a byte");
  require(zero.next_sample_ticks() == start_center, "Independent first start-bit center is tick48");
  require(!zero.observe(stop, true) && zero.next_sample_ticks() == completed,
          "Nine low symbols leave only the independent stop center pending");
  require(zero.observe(completed, true) == MidiSerialByte{.value = 0,
                                                          .start_ticks = start,
                                                          .stop_sample_ticks = completed} &&
              zero.received_bytes() == 1 && !zero.next_sample_ticks(),
          "Literal all-zero wire produces00 at the stop center, not at stop entry");
  require(!zero.observe(completed, true), "Identical repeated observations are inert");
  MidiSerialReceiver ones(hz);
  static_cast<void>(ones.observe(0, true));
  static_cast<void>(ones.observe(start, false));
  static_cast<void>(ones.observe(64, true));
  require(ones.observe(completed, true) == MidiSerialByte{.value = all_high,
                                                          .start_ticks = start,
                                                          .stop_sample_ticks = completed},
          "Literal one-bit start followed by high data/stop producesFF");
  constexpr unsigned expected = 0x55;
  for (std::size_t cut = 0; cut < alternating.size(); ++cut) {
    MidiSerialReceiver original(hz);
    static_cast<void>(original.observe(0, true));
    for (std::size_t i = 0; i <= cut; ++i) {
      require(!original.observe(alternating[i].ticks, alternating[i].high),
              "Alternating data cannot complete before its stop center");
    }
    auto copy = original;
    for (std::size_t i = cut + 1; i < alternating.size(); ++i) {
      require(!original.observe(alternating[i].ticks, alternating[i].high) &&
                  !copy.observe(alternating[i].ticks, alternating[i].high),
              "Both actual partial copies retain their expected bit positions");
    }
    const MidiSerialByte endpoint{
        .value = expected, .start_ticks = start, .stop_sample_ticks = completed};
    require(original.observe(completed, true) == endpoint &&
                copy.observe(completed, true) == endpoint && copy.received_bytes() == 1,
            "Ten partial-state cuts reproduce literal55 and its exact stop timestamp");
  }
}

void all_octets() {
  MidiSerialReceiver receiver(hz);
  static_cast<void>(receiver.observe(0, true));
  constexpr unsigned words = 256;
  constexpr unsigned data_bits = 8;
  constexpr unsigned frame_bits = 10;
  constexpr unsigned spacing = next_start;
  for (unsigned value = 0; value < words; ++value) {
    const auto at = std::uint64_t{start} + (std::uint64_t{value} * spacing);
    require(!receiver.observe(at, false), "Start of each independent raw octet is not a byte");
    for (unsigned i = 0; i < data_bits; ++i) {
      require(!receiver.observe(at + ((i + 1) * bit), (value & (1U << i)) != 0),
              "Wire data bits remain incomplete until stop sampling");
    }
    require(!receiver.observe(at + ((frame_bits - 1) * bit), true),
            "A rising stop edge alone cannot publish an octet");
    require(receiver.observe(at + (frame_bits * bit), true) ==
                MidiSerialByte{.value = static_cast<std::uint8_t>(value),
                               .start_ticks = at,
                               .stop_sample_ticks = at + (completed - start)},
            "All256 raw octets, including non-message data/status values, preserve every bit");
  }
  require(receiver.received_bytes() == words, "No MIDI semantic filtering occurs in the wire sink");
}

void boundaries_and_back_to_back() {
  MidiSerialReceiver tied(hz);
  static_cast<void>(tied.observe(0, true));
  static_cast<void>(tied.observe(start, false));
  require(!tied.observe(data_zero_center, true),
          "A data transition at its center samples the new level");
  static_cast<void>(tied.observe(data_one_edge, false));
  static_cast<void>(tied.observe(stop, true));
  require(tied.observe(completed, true) ==
              MidiSerialByte{.value = 1, .start_ticks = start, .stop_sample_ticks = completed},
          "Literal center-tied data0 is one; later data bits are zero");
  MidiSerialReceiver continuous(hz);
  static_cast<void>(continuous.observe(0, true));
  static_cast<void>(continuous.observe(start, false));
  static_cast<void>(continuous.observe(stop, true));
  require(continuous.observe(next_start, false) == MidiSerialByte{.value = 0,
                                                                  .start_ticks = start,
                                                                  .stop_sample_ticks = completed} &&
              continuous.next_sample_ticks() == next_start_center,
          "One observation retires the prior stop and captures the next contiguous start");
  static_cast<void>(continuous.observe(next_data_zero_edge, true));
  require(continuous.observe(next_completed, true) ==
                  MidiSerialByte{.value = all_high,
                                 .start_ticks = next_start,
                                 .stop_sample_ticks = next_completed} &&
              continuous.received_bytes() == 2,
          "Back-to-back00/FF frames need no extra idle bit or forced stop-boundary service");
}

void fractional_clock() {
  constexpr std::uint64_t fractional_hz = 1000001;
  MidiSerialReceiver receiver(fractional_hz);
  static_cast<void>(receiver.observe(0, true));
  static_cast<void>(receiver.observe(start, false));
  require(receiver.next_sample_ticks() == fractional_start_center,
          "Fractional half-bit center rounds to tick49");
  require(!receiver.observe(fractional_stop, true) &&
              receiver.next_sample_ticks() == fractional_completed,
          "Fractional scheduling does not accumulate a rounded whole-bit duration");
  require(receiver.observe(fractional_completed, true) ==
              MidiSerialByte{
                  .value = 0, .start_ticks = start, .stop_sample_ticks = fractional_completed},
          "Independent fractional-clock all-zero stop center is tick337");
}

void refusals() {
  rejects([] { static_cast<void>(MidiSerialReceiver(1)); }, "timebase");
  rejects([] { static_cast<void>(MidiSerialReceiver(std::numeric_limits<std::uint64_t>::max())); },
          "timebase");
  MidiSerialReceiver stop_low(hz);
  static_cast<void>(stop_low.observe(0, true));
  static_cast<void>(stop_low.observe(start, false));
  rejects([&] { static_cast<void>(stop_low.observe(completed, false)); }, "stop bit");
  require(stop_low.faulted() && stop_low.received_bytes() == 0,
          "Invalid stop cannot invent a byte");
  rejects([&] { static_cast<void>(stop_low.observe(completed + 1, true)); }, "recovery");
  MidiSerialReceiver start_high(hz);
  static_cast<void>(start_high.observe(0, true));
  static_cast<void>(start_high.observe(start, false));
  static_cast<void>(start_high.observe(early_start_glitch, true));
  rejects([&] { static_cast<void>(start_high.observe(start_center, true)); }, "start bit");
  MidiSerialReceiver unknown(hz);
  static_cast<void>(unknown.observe(0, std::nullopt));
  static_cast<void>(unknown.observe(10, false));
  require(!unknown.next_sample_ticks(), "Unknown-to-low does not invent a start edge");
  static_cast<void>(unknown.observe(idle_recovery, true));
  static_cast<void>(unknown.observe(start, false));
  rejects([&] { static_cast<void>(unknown.observe(early_start_glitch, std::nullopt)); },
          "unknown active");
  MidiSerialReceiver backwards(hz);
  static_cast<void>(backwards.observe(start, true));
  rejects([&] { static_cast<void>(backwards.observe(before_start, true)); }, "nonmonotonic");
  MidiSerialReceiver tied(hz);
  static_cast<void>(tied.observe(start, true));
  rejects([&] { static_cast<void>(tied.observe(start, false)); }, "same-tick");
  MidiSerialReceiver overflow(hz);
  constexpr auto end = std::numeric_limits<std::uint64_t>::max();
  static_cast<void>(overflow.observe(end - 1, true));
  rejects([&] { static_cast<void>(overflow.observe(end, false)); }, "deadline overflow");
}

void native_transmitter() {
  constexpr unsigned bus_hz = 30000000;
  constexpr unsigned divisor = 60;
  constexpr unsigned baud_high = 0xc8;
  constexpr unsigned baud_low = 0xc9;
  constexpr unsigned control2 = 0xcb;
  constexpr unsigned status1 = 0xcc;
  constexpr unsigned data_low = 0xcf;
  constexpr unsigned enabled = 8;
  constexpr unsigned complete = 0x40;
  constexpr unsigned maximum_cycles = 30000;
  constexpr std::array<std::uint8_t, 3> words{0, 0xa5, all_high};
  McuSci sci;
  sci.write({.address = baud_high, .value = 0});
  sci.write({.address = baud_low, .value = divisor});
  sci.write({.address = control2, .value = enabled});
  MidiSerialReceiver receiver(bus_hz);
  std::uint64_t ticks = 0;
  static_cast<void>(receiver.observe(ticks, sci.transmit_level()));
  for (const auto value : words) {
    static_cast<void>(sci.read(status1));
    sci.write({.address = data_low, .value = value});
    std::optional<MidiSerialByte> received;
    unsigned cycles = 0;
    for (; cycles < maximum_cycles; ++cycles) {
      sci.advance({.bus_cycles = 1});
      ++ticks;
      if (const auto frame = receiver.observe(ticks, sci.transmit_level())) {
        require(!received && frame->value == value,
                "Native SCI produces exactly the requested octet");
        received = frame;
      }
      if (received && (sci.read(status1) & complete) != 0) { break; }
    }
    require(cycles < maximum_cycles && received && sci.transmit_level() == true,
            "Actual SCI preamble, data and final stop finish before native TC");
  }
  require(receiver.received_bytes() == words.size(),
          "Three real SCI frames reach the passive sink");
}
}

int main() {
  try {
    literals();
    all_octets();
    boundaries_and_back_to_back();
    fractional_clock();
    refusals();
    native_transmitter();
    std::cout << "PASS literal00/FF/55, ten cuts,256 raw octets, center ties, contiguous frames, "
                 "fractional clock, refusals and3 "
                 "actual SCI frames; board/host output unqualified\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
