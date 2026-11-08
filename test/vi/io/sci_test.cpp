// SPDX-License-Identifier: GPL-3.0-only
#include "io/sci.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {
using blofeld::vi::Sci;
constexpr std::uint16_t baud = 0xc8, control1 = 0xca, control2 = 0xcb;
constexpr std::uint16_t status = 0xcc, status2 = 0xcd, data_high = 0xce, data = 0xcf;
constexpr std::uint16_t ready = 0xc0, empty = 0x80, full = 0x20, overrun = 8, idle = 0x10;
constexpr std::uint16_t rx_irq_enable = 0x24, uart_vector = 0xffd6;
constexpr std::uint16_t note_on = 0x90, velocity = 0x3c, all_bits = 0xff;
constexpr Sci::Cycle frame = 160, transfer = 153;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(Sci &sci, const Sci::Write access) {
  require(sci.write(access), "Write a supported UF32 SCI register");
}

std::uint16_t read(Sci &sci, const Sci::Read access) {
  std::uint16_t value = 0;
  require(sci.read(access, value), "Read a defined UF32 SCI register");
  return value;
}

void setup(Sci &sci, const std::uint16_t mode) {
  write(sci, {.offset = baud, .value = 1, .bytes = 2});
  write(sci, {.offset = control2, .value = mode});
}

void reset_and_baud() {
  Sci sci;
  require(read(sci, {.offset = baud, .bytes = 2}) == 4 && read(sci, {.offset = status}) == ready,
          "Reset has baud divisor four and empty/complete transmit flags");
  write(sci, {.offset = baud, .value = all_bits});
  std::uint16_t value = 0;
  require(!sci.read({.offset = baud}, value),
          "High-only baud readback remains explicitly undefined");
  write(sci, {.offset = baud + 1, .value = 1});
  constexpr std::uint16_t committed = 0x1f01;
  require(read(sci, {.offset = baud, .bytes = 2}) == committed,
          "The low byte commits the staged high byte; UF32 has no infrared bits");
  write(sci, {.offset = status, .value = 0});
  require(read(sci, {.offset = status}) == ready, "Status is read-only, not write-one-to-clear");
  write(sci, {.offset = status2, .value = all_bits});
  require(read(sci, {.offset = status2}) == 10, "Only UF32 BRK13/TXDIR control bits are writable");
  write(sci, {.offset = data_high, .value = all_bits});
  require(read(sci, {.offset = data_high}) == 0x40, "T8 is retained and writing R8 has no effect");
  require(!sci.read({.offset = baud - 1}, value) && !sci.write({.offset = data + 1}) &&
              !sci.write({.offset = control1, .value = 0x80}) &&
              !sci.write({.offset = control2, .value = 1}),
          "Unknown registers and unbound loop/break paths fail explicitly");
}

void transmit_pipeline() {
  Sci sci;
  setup(sci, 8);
  require(read(sci, {.offset = status}) == empty && sci.deadline() == transfer,
          "TE queues a real preamble and clears transmission complete");
  write(sci, {.offset = data, .value = note_on});
  require(sci.advance(transfer - 1) && (read(sci, {.offset = status}) & empty) == 0,
          "The data buffer stays occupied before the documented stop-bit transfer");
  require(sci.advance(transfer) && read(sci, {.offset = status}) == empty,
          "TDRE rises at 9/16 of the preamble's stop bit");
  write(sci, {.offset = data, .value = velocity});
  require(sci.advance(frame) && !sci.take_output(), "Preambles are not fabricated MIDI bytes");
  require(sci.advance((2 * frame) - 1) && !sci.take_output(),
          "A character cannot be consumed before its complete stop bit");
  require(sci.advance(2 * frame), "Complete the first actual character");
  const auto first = sci.take_output();
  require(first && first->cycle == 2 * frame && first->word == note_on && first->bits == 8,
          "The first emitted byte retains its original value, width and completion time");
  require(sci.advance(3 * frame), "Complete the buffered next character");
  const auto second = sci.take_output();
  require(second && second->cycle == 3 * frame && second->word == velocity && !sci.take_output() &&
              read(sci, {.offset = status}) == ready && read(sci, {.offset = data}) == 0,
          "Buffered characters are contiguous; TX and RX registers remain distinct");

  Sci unobserved;
  write(unobserved, {.offset = data, .value = velocity});
  setup(unobserved, 8);
  require(unobserved.advance(frame) && !unobserved.take_output(),
          "Writing SCIDRL without first observing TDRE does not start a new transmission");
}

void receive_flags_and_overrun() {
  Sci sci;
  setup(sci, rx_irq_enable);
  require(sci.receive({.word = note_on}) && (read(sci, {.offset = status2}) & 1) != 0 &&
              sci.advance(frame - 1) && !sci.pending(),
          "RAF is active during a character, before RDRF and its IRQ become eligible");
  require(sci.advance(frame) && sci.pending() == uart_vector,
          "A complete received character requests the SCI interrupt");
  require(read(sci, {.offset = data}) == note_on && read(sci, {.offset = status}) == (ready | full),
          "A data-only read cannot clear an unobserved RDRF flag");
  require(sci.receive({.word = empty, .errors = 6}) && sci.advance(2 * frame),
          "Receive another character while the previous byte remains unread");
  require(read(sci, {.offset = data}) == note_on && sci.pending() == uart_vector,
          "An overrun loses the incoming word and preserves the previous one");
  require(read(sci, {.offset = status}) == (ready | overrun),
          "An overrun that occurred after the status read survives the data read");
  require(read(sci, {.offset = data}) == note_on && !sci.pending(),
          "A fresh status/data sequence clears OR and its IRQ");
  require(sci.receive({.word = velocity}) && sci.advance(3 * frame) &&
              read(sci, {.offset = status}) == (ready | full) &&
              read(sci, {.offset = data}) == velocity,
          "Reception resumes without inheriting errors from the lost overrun frame");
  constexpr Sci::Cycle idle_at = 624;
  require(sci.advance(idle_at) && (read(sci, {.offset = status2}) & 1) == 0,
          "RAF clears after the actual trailing idle character");
  constexpr std::uint16_t idle_interrupt = 0x14;
  write(sci, {.offset = control2, .value = idle_interrupt});
  require(sci.pending() == uart_vector && read(sci, {.offset = status}) == (ready | idle),
          "The retained IDLE flag can request its separately enabled interrupt");
  static_cast<void>(read(sci, {.offset = data}));
  require(sci.advance(idle_at * 2) && !sci.pending() && !sci.deadline(),
          "Cleared IDLE cannot repeatedly rearm without a new received character");
}

void parity_and_wakeup() {
  Sci sci;
  constexpr std::uint16_t nine_even = 0x12;
  constexpr std::uint16_t datum = 0x55;
  constexpr std::uint16_t bad_parity = 0x155;
  constexpr Sci::Cycle wide_frame = 176;
  write(sci, {.offset = control1, .value = nine_even});
  setup(sci, rx_irq_enable);
  require(sci.receive({.word = datum}) && sci.advance(wide_frame) &&
              read(sci, {.offset = status}) == (ready | full) &&
              read(sci, {.offset = data}) == datum,
          "Nine-bit reception checks parity across the actual received frame");
  require(sci.receive({.word = bad_parity}) && sci.advance(2 * wide_frame) &&
              read(sci, {.offset = status}) == (ready | full | 1),
          "An incorrect ninth parity bit raises PF alongside RDRF");
  constexpr std::uint16_t high_and_low = 0x8055;
  require(read(sci, {.offset = data_high, .bytes = 2}) == high_and_low,
          "A word read captures R8 before SCIDRL clears the observed flags");

  Sci transmitter;
  write(transmitter, {.offset = control1, .value = nine_even});
  setup(transmitter, 8);
  write(transmitter, {.offset = data_high, .value = 0x40});
  static_cast<void>(read(transmitter, {.offset = status}));
  write(transmitter, {.offset = data, .value = datum});
  require(transmitter.advance(2 * wide_frame), "Complete a nine-bit parity-protected transmission");
  const auto sent = transmitter.take_output();
  require(sent && sent->word == datum && sent->bits == 9 && sent->cycle == 2 * wide_frame &&
              read(transmitter, {.offset = data_high}) == 0x40,
          "The generated parity bit replaces T8 in the frame without changing its register latch");

  Sci address;
  write(address, {.offset = control1, .value = 8});
  setup(address, rx_irq_enable | 2);
  require(address.receive({.word = 1}) && address.advance(frame) && !address.pending(),
          "Address-mark standby suppresses ordinary data characters");
  require(address.receive({.word = note_on}) && address.advance(2 * frame) &&
              address.pending() == uart_vector && (read(address, {.offset = control2}) & 2) == 0,
          "An address character clears RWU and becomes available to the receiver ISR");

  Sci idle_wake;
  setup(idle_wake, rx_irq_enable | 2);
  require(idle_wake.receive({.word = note_on}) && idle_wake.advance(frame) && !idle_wake.pending(),
          "Idle-line standby suppresses characters until the actual idle condition");
  constexpr Sci::Cycle idle_wakeup = 288;
  require(idle_wake.advance(idle_wakeup) && (read(idle_wake, {.offset = control2}) & 2) == 0 &&
              read(idle_wake, {.offset = status}) == ready,
          "Idle wakeup clears RWU without inventing an IDLE interrupt");
}

void wait_gating_and_disabled_clock() {
  Sci sci;
  setup(sci, 8);
  constexpr Sci::Cycle pause_start = 80;
  constexpr Sci::Cycle pause_end = 1080;
  require(sci.advance(pause_start), "Advance halfway through the preamble");
  write(sci, {.offset = control1, .value = 0x40});
  sci.activity({.waiting = true});
  require(!sci.deadline() && sci.advance(pause_end),
          "SCISWAI freezes deadlines and transfer phase");
  sci.activity({});
  require(sci.deadline() == pause_end + transfer - pause_start &&
              sci.advance(pause_end + frame - pause_start) &&
              read(sci, {.offset = status}) == ready,
          "Resume preserves the remaining character duration");

  Sci disabled;
  write(disabled, {.offset = baud, .value = 0, .bytes = 2});
  write(disabled, {.offset = control2, .value = 8});
  require(!disabled.deadline() && disabled.advance(frame),
          "A zero baud divisor stops the generator");
  write(disabled, {.offset = baud, .value = 1, .bytes = 2});
  require(disabled.deadline() == frame + transfer,
          "Enabling the generator starts the pending preamble instead of fabricating completion");
}

void midi_baud_and_framing() {
  Sci sci;
  constexpr std::uint16_t midi_divisor = 60;
  constexpr Sci::Cycle midi_character = 9600; // 320 us at the 30 MHz UF32 bus clock.
  write(sci, {.offset = baud, .value = midi_divisor, .bytes = 2});
  write(sci, {.offset = control2, .value = rx_irq_enable});
  require(sci.receive({.word = note_on}) && sci.advance(midi_character - 1) && !sci.pending(),
          "MIDI uses the programmed divisor, with no early character completion");
  require(sci.advance(midi_character) && read(sci, {.offset = status}) == (ready | full) &&
              read(sci, {.offset = data}) == note_on,
          "A 31250-baud eight-bit character occupies exactly 9600 nominal bus cycles");
  require(sci.receive({.word = 0, .errors = 2}) && sci.advance(2 * midi_character),
          "Receive a framing-error character");
  require(sci.receive({.word = velocity}) && read(sci, {.offset = status}) == (ready | full | 2) &&
              read(sci, {.offset = data}) == 0,
          "FE inhibits a later receive until the observed framing error is cleared");
  require(sci.receive({.word = velocity}) && sci.advance(3 * midi_character) &&
              read(sci, {.offset = status}) == (ready | full) &&
              read(sci, {.offset = data}) == velocity,
          "Reception resumes after the documented status/data clear sequence");
}

void bounded_output_and_limits() {
  Sci sci;
  setup(sci, 8);
  require(sci.advance(frame), "Finish the initial preamble");
  Sci::Cycle now = frame;
  constexpr Sci::Cycle idle_and_frame = frame + 16;
  for (std::size_t index = 0; index < Sci::output_capacity; ++index) {
    static_cast<void>(read(sci, {.offset = status}));
    write(sci, {.offset = data, .value = static_cast<std::uint16_t>(index)});
    now += idle_and_frame;
    require(sci.advance(now), "Fill the bounded completed-character output queue");
  }
  static_cast<void>(read(sci, {.offset = status}));
  write(sci, {.offset = data, .value = note_on});
  require(!sci.advance(now + idle_and_frame) && sci.error() != nullptr &&
              std::string_view{sci.error()} == "SCI output queue is full",
          "An undrained consumer fails visibly rather than losing or replacing output");
  for (std::size_t index = 0; index < Sci::output_capacity; ++index) {
    const auto output = sci.take_output();
    require(output && output->word == index &&
                output->cycle == frame + ((index + 1) * idle_and_frame),
            "Overflow preserves every previously completed character and its timestamp");
  }
  require(!sci.take_output(), "No fabricated output follows the retained characters");
  Sci limit;
  require(limit.advance(std::numeric_limits<Sci::Cycle>::max()) && !limit.advance(0),
          "Idle spans are constant-time and backward time fails explicitly");
}
}

int main() try {
  reset_and_baud();
  transmit_pipeline();
  receive_flags_and_overrun();
  parity_and_wakeup();
  wait_gating_and_disabled_clock();
  midi_baud_and_framing();
  bounded_output_and_limits();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
