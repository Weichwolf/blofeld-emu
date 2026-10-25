// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_sci.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
// Literal expectations from S12SCIV4 sections 3, 4.5 and 4.6, together with
// the reduced MC9S12UF32 register map and 64/100-pin routing tables.
constexpr unsigned baud_high = 0xc8;
constexpr unsigned baud_low = 0xc9;
constexpr unsigned control1 = 0xca;
constexpr unsigned control2 = 0xcb;
constexpr unsigned status1 = 0xcc;
constexpr unsigned status2 = 0xcd;
constexpr unsigned data_high = 0xce;
constexpr unsigned data_low = 0xcf;
constexpr unsigned empty = 0x80;
constexpr unsigned idle_transmitter = 0xc0;
constexpr unsigned data_available = 0xe0;
constexpr unsigned pattern = 0x55;
constexpr unsigned inverse_pattern = 0xaa;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned midi_divider = 60;
constexpr unsigned frame_ticks = 160;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

void configure(blofeld::McuSci &sci, const std::uint8_t direction) {
  sci.write({.address = baud_high, .value = 0});
  sci.write({.address = baud_low, .value = 1});
  sci.write({.address = control2, .value = direction});
}

void samples(blofeld::McuSci &sci, const bool level, const unsigned count) {
  sci.advance({.bus_cycles = count, .receive_level = level});
}

struct WireFrame {
  unsigned data = 0;
  unsigned width = 8;
  bool stop = true;
};

void wire_frame(blofeld::McuSci &sci, const WireFrame frame) {
  if (frame.width != 8 && frame.width != 9) { throw blofeld::Fault("Invalid test frame width"); }
  samples(sci, true, 3);
  samples(sci, false, 16);
  for (unsigned bit = 0; bit < frame.width; ++bit) {
    samples(sci, ((frame.data >> bit) & 1U) != 0, 16);
  }
  samples(sci, frame.stop, 16);
}

void test_reset_latches_and_read_sequences() {
  blofeld::McuSci sci;
  constexpr std::array<unsigned, 8> reset{0, 4, 0, 0, 0xc0, 0, 0, 0};
  for (unsigned index = 0; index < reset.size(); ++index) {
    require(sci.read(static_cast<std::uint16_t>(baud_high + index)) == reset.at(index),
            "SCI reset register values");
  }
  require(!sci.transmit_level(), "Disabled transmitter does not own its GPIO pad");
  sci.write({.address = baud_high, .value = byte_mask});
  reject([&sci] { static_cast<void>(sci.read(baud_high)); },
         "Guide leaves high-byte readback unspecified before SCIBDL commit");
  constexpr unsigned low_divider = 0x23;
  sci.write({.address = baud_low, .value = low_divider});
  constexpr unsigned high_mask = 0x1f;
  require(sci.read(baud_high) == high_mask && sci.read(baud_low) == low_divider,
          "UF32 reserves the IR and narrow-pulse baud bits");
  sci.write({.address = status2, .value = byte_mask});
  require(sci.read(status2) == 10, "UF32 has no TXPOL/RXPOL fields");
  sci.write({.address = status1, .value = byte_mask});
  require(sci.read(status1) == idle_transmitter, "Status writes do not clear SCI flags");
  sci.reset();
  sci.write({.address = data_low, .value = pattern}); // No status read after reset.
  require(sci.read(status1) == idle_transmitter && sci.read(data_low) == 0,
          "Unarmed transmit writes do not launch data or change receive data");
}

void test_transmit_preamble_and_wire_order() {
  blofeld::McuSci sci;
  sci.write({.address = baud_high, .value = 0});
  sci.write({.address = baud_low, .value = midi_divider});
  sci.write({.address = control2, .value = 8});
  require(sci.read(status1) == empty && sci.transmit_level() == true,
          "TE queues a full idle preamble and clears TC");
  sci.write({.address = data_low, .value = pattern});
  require(sci.read(status1) == 0, "Status read then SCIDRL write clears TDRE");
  constexpr unsigned before_load = 152;
  sci.advance({.bus_cycles = before_load * midi_divider});
  require(sci.read(status1) == 0, "Data stays buffered before 9/16 of the stop bit");
  sci.advance({.bus_cycles = midi_divider - 1});
  require(sci.read(status1) == 0, "Baud divider preserves sub-tick bus-cycle phase");
  sci.advance({.bus_cycles = 1});
  require(sci.read(status1) == empty && sci.transmit_level() == true,
          "TDRE rises on shift transfer at 9/16 while stop/preamble stays high");
  sci.advance({.bus_cycles = 7 * midi_divider});
  constexpr std::array<bool, 10> expected{
      false, true, false, true, false, true, false, true, false, true};
  for (const bool level : expected) {
    require(sci.transmit_level() == level, "Start, LSB-first 55 data, and stop waveform");
    sci.advance({.bus_cycles = 16 * midi_divider});
  }
  require(sci.read(status1) == idle_transmitter && sci.transmit_level() == true,
          "TC rises only after the complete final stop bit");
  sci.write({.address = control2, .value = 0});
  require(!sci.transmit_level(), "TE release after TC restores GPIO ownership");
}

void test_back_to_back_and_disable_during_transmission() {
  blofeld::McuSci sci;
  configure(sci, 8);
  static_cast<void>(sci.read(status1));
  sci.write({.address = data_low, .value = pattern});
  sci.advance({.bus_cycles = frame_ticks});
  static_cast<void>(sci.read(status1));
  sci.write({.address = data_low, .value = inverse_pattern});
  sci.advance({.bus_cycles = frame_ticks});
  require(sci.transmit_level() == false, "Next frame starts without an invented idle gap");
  sci.write({.address = control2, .value = 0});
  constexpr std::array<bool, 10> expected{
      false, false, true, false, true, false, true, false, true, true};
  for (const bool level : expected) {
    require(sci.transmit_level() == level, "Clearing TE preserves the current frame");
    sci.advance({.bus_cycles = 16});
  }
  require(!sci.transmit_level() && sci.read(status1) == idle_transmitter,
          "Completed frame releases the disabled TX pad");
}

void test_queued_preamble() {
  blofeld::McuSci sci;
  configure(sci, 8);
  static_cast<void>(sci.read(status1));
  sci.write({.address = data_low, .value = pattern});
  sci.advance({.bus_cycles = frame_ticks});
  sci.write({.address = control2, .value = 0});
  sci.write({.address = control2, .value = 8});
  static_cast<void>(sci.read(status1));
  sci.write({.address = data_low, .value = inverse_pattern});
  sci.advance({.bus_cycles = frame_ticks});
  require(sci.transmit_level() == true && sci.read(status1) == 0,
          "A TE toggle queues an idle character before the next buffered data");
  constexpr unsigned transfer_ticks = 153;
  sci.advance({.bus_cycles = transfer_ticks});
  require(sci.transmit_level() == true && sci.read(status1) == empty,
          "Queued preamble holds all mark bits while the next data is loaded");
  sci.advance({.bus_cycles = 7});
  require(sci.transmit_level() == false, "Next frame starts after the complete queued preamble");
}

void test_receive_all_bytes_and_ninth_bit() {
  for (unsigned value = 0; value < 256; ++value) {
    blofeld::McuSci sci;
    configure(sci, 4);
    wire_frame(sci, {.data = value});
    require(sci.read(data_low) == value, "All eight data bits reach receive register");
    require(sci.read(status1) == data_available, "Unarmed data reads preserve RDRF");
    static_cast<void>(sci.read(data_high));
    require(sci.read(status1) == data_available, "SCIDRH does not acknowledge RDRF");
    require(sci.read(data_low) == value && sci.read(status1) == idle_transmitter,
            "Status then SCIDRL read acknowledges RDRF");
  }
  blofeld::McuSci sci;
  sci.write({.address = control1, .value = 16});
  configure(sci, 4);
  sci.write({.address = data_high, .value = 0x40});
  constexpr unsigned nine_bits = 0x1a5;
  wire_frame(sci, {.data = nine_bits, .width = 9});
  require(sci.read(data_high) == idle_transmitter, "R8 and persistent T8 are separate high bits");
  static_cast<void>(sci.read(status1));
  constexpr unsigned low_bits = 0xa5;
  require(sci.read(data_low) == low_bits, "Nine-bit frame retains low eight bits");
}

void test_overrun_and_framing_error_inhibition() {
  blofeld::McuSci sci;
  configure(sci, 4);
  wire_frame(sci, {.data = pattern});
  static_cast<void>(sci.read(status1)); // Only RDRF observed; OR has not happened.
  wire_frame(sci, {.data = inverse_pattern, .stop = false});
  require(sci.read(data_low) == pattern, "Overrun retains the older unread word");
  constexpr unsigned overrun_only = 0xc8;
  require(sci.read(status1) == overrun_only, "New unobserved OR persists; overrun suppresses FE");
  static_cast<void>(sci.read(data_low));
  wire_frame(sci, {.stop = false});
  constexpr unsigned framing = 0xe2;
  require(sci.read(status1) == framing, "Zero stop bit sets RDRF and FE");
  wire_frame(sci, {.data = inverse_pattern});
  require(sci.read(data_low) == 0, "FE inhibits further frame reception until acknowledged");
  wire_frame(sci, {.data = inverse_pattern});
  require(sci.read(status1) == data_available && sci.read(data_low) == inverse_pattern,
          "Status/data acknowledgement restores reception after FE");
}

void test_parity_and_transmit_ninth_bit() {
  struct Case {
    std::uint8_t format = 0;
    unsigned width = 0;
    unsigned wire_word = 0;
  };

  constexpr std::array<Case, 5> cases{{{.format = 0x12, .width = 9, .wire_word = 0x107},
                                       {.format = 0x13, .width = 9, .wire_word = 0x007},
                                       {.format = 0x02, .width = 8, .wire_word = 0x087},
                                       {.format = 0x03, .width = 8, .wire_word = 0x007},
                                       {.format = 0x10, .width = 9, .wire_word = 0x107}}};
  for (const auto &item : cases) {
    if (item.width != 8 && item.width != 9) { throw blofeld::Fault("Invalid parity test width"); }
    blofeld::McuSci sci;
    sci.write({.address = control1, .value = item.format});
    configure(sci, 8);
    sci.write({.address = data_high, .value = 0x40});
    static_cast<void>(sci.read(status1));
    sci.write({.address = data_low, .value = 7});
    sci.advance({.bus_cycles = (item.width + 2) * 16});
    require(sci.transmit_level() == false, "Parity/9-bit frame begins with a start bit");
    sci.advance({.bus_cycles = 16});
    for (unsigned bit = 0; bit < item.width; ++bit) {
      require(sci.transmit_level() == (((item.wire_word >> bit) & 1U) != 0),
              "Independent even/odd parity and T8 wire vectors");
      sci.advance({.bus_cycles = 16});
    }
    require(sci.transmit_level() == true, "Parity/9-bit frame ends with a stop bit");
    sci.reset();
    sci.write({.address = control1, .value = item.format});
    configure(sci, 4);
    wire_frame(sci, {.data = item.wire_word, .width = item.width});
    require(sci.read(status1) == data_available, "Reference frame has correct parity");
    require(sci.read(data_high) == ((item.wire_word >> 1U) & empty) &&
                sci.read(data_low) == (item.wire_word & byte_mask),
            "Receive shift bits reach SCIDR alongside parity checking");
    if ((item.format & 2U) != 0) {
      const auto parity_mask = item.width == 9 ? 0x100U : 0x80U;
      const auto bad_word = item.wire_word ^ parity_mask;
      wire_frame(sci, {.data = bad_word, .width = item.width});
      constexpr unsigned parity_failed = 0xe1;
      require(sci.read(status1) == parity_failed, "Flipped parity bit raises PF");
    }
  }
}

void test_start_and_stop_sample_tables() {
  for (unsigned pattern_bits = 0; pattern_bits < 8; ++pattern_bits) {
    blofeld::McuSci sci;
    configure(sci, 4);
    samples(sci, true, 3);
    for (unsigned tick = 1; tick <= 16; ++tick) {
      const bool verification = tick == 3 || tick == 5 || tick == 7;
      const bool level = verification && ((pattern_bits >> ((tick - 3) / 2)) & 1U) != 0;
      samples(sci, level, 1);
    }
    samples(sci, false, 8 * 16);
    samples(sci, true, 16);
    const auto ones = std::popcount(pattern_bits);
    if (ones <= 1) {
      const auto expected = data_available | (ones == 0 ? 0U : 4U);
      require(sci.read(status1) == expected, "RT3/5/7 verification and start-noise table");
    } else {
      require((sci.read(status1) & 0x20U) == 0, "Rejected start cannot generate a receive word");
    }
    sci.reset();
    configure(sci, 4);
    samples(sci, true, 3);
    samples(sci, false, 9 * 16);
    for (unsigned tick = 1; tick <= 16; ++tick) {
      const bool middle = tick >= 8 && tick <= 10;
      const bool level = !middle || ((pattern_bits >> (tick - 8)) & 1U) != 0;
      samples(sci, level, 1);
    }
    const auto expected =
        data_available | (ones >= 2 ? 0U : 2U) | (ones == 0 || ones == 3 ? 0U : 4U);
    require(sci.read(status1) == expected, "RT8/9/10 stop-majority, framing and noise table");
  }
}

void test_data_majorities_and_noise() {
  for (unsigned pattern_bits = 0; pattern_bits < 8; ++pattern_bits) {
    blofeld::McuSci sci;
    configure(sci, 4);
    samples(sci, true, 3);
    samples(sci, false, 16);
    for (unsigned tick = 1; tick <= 16; ++tick) {
      const bool level = tick >= 8 && tick <= 10 && ((pattern_bits >> (tick - 8)) & 1U) != 0;
      samples(sci, level, 1);
    }
    samples(sci, false, 7 * 16);
    samples(sci, true, 16);
    const auto ones = std::popcount(pattern_bits);
    const unsigned expected = ones >= 2 ? 1 : 0;
    const unsigned expected_status = data_available | (ones == 0 || ones == 3 ? 0U : 4U);
    require(sci.read(status1) == expected_status && sci.read(data_low) == expected,
            "RT8/9/10 data majority and noise match all eight reference table rows");
  }
}

void test_idle_and_unknown_transfer_interval() {
  blofeld::McuSci sci;
  configure(sci, 4);
  samples(sci, true, frame_ticks);
  constexpr unsigned idle_status = 0xd0;
  require(sci.read(status1) == idle_status, "Ten consecutive mark bits detect initial idle");
  static_cast<void>(sci.read(data_low));
  samples(sci, true, frame_ticks);
  require(sci.read(status1) == idle_transmitter, "Cleared IDLE needs another received frame");
  samples(sci, false, 16);
  samples(sci, false, 8 * 16);
  samples(sci, true, 10);
  reject([&sci] { static_cast<void>(sci.read(status1)); },
         "Unverified stop-to-flag phase cannot silently assert a ready flag");
  reject([&sci] { static_cast<void>(sci.read(data_low)); },
         "Unverified transfer phase also protects receive data");
  samples(sci, true, 6);
  require(sci.read(status1) == data_available, "A completed frame is readable after stop");
  static_cast<void>(sci.read(data_low));
  samples(sci, true, frame_ticks);
  require(sci.read(status1) == idle_status && sci.read(status2) == 0,
          "A new frame rearms IDLE and idle clears RAF");
}

void test_unknown_input_and_unsupported_modes() {
  blofeld::McuSci sci;
  configure(sci, 4);
  reject([&sci] { sci.advance({.bus_cycles = 1}); }, "Unknown RX cannot be assumed idle");
  constexpr std::array<std::uint8_t, 2> unsupported{1, 2};
  for (const auto value : unsupported) {
    reject([&sci, value] { sci.write({.address = control2, .value = value}); },
           "Unsupported control modes must fail");
    require(sci.read(control2) == 4, "Rejected control writes preserve current configuration");
  }
  reject([&sci] { sci.write({.address = control1, .value = 0x80}); },
         "Loop and single-wire mode cannot act as normal RX");
  sci.write({.address = baud_low, .value = 0});
  sci.advance({.bus_cycles = frame_ticks});
  require(sci.read(status1) == idle_transmitter, "Zero divisor disables baud generation");
  sci.reset();
  require(sci.read(baud_low) == 4 && sci.read(control2) == 0, "Reset aborts all SCI activity");
}

void test_bus_pin_routing() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus bus(flash);
  constexpr unsigned port_r_input = 0x269;
  constexpr unsigned port_s_input = 0x271;
  constexpr unsigned routing = 0x257;
  constexpr unsigned s_directions = 0x272;
  constexpr unsigned s_latch = 0x270;
  constexpr unsigned r_external = 0xf7;
  constexpr unsigned s_external = 0xdf;
  bus.set_pin_drive(blofeld::McuPort::r, {.mask = r_external, .levels = 4});
  bus.write(control2, 12);
  require(bus.read(port_r_input) == 12, "100-pin SCI TX/RX override GPIO direction on PR3/2");
  bus.set_pin_drive(blofeld::McuPort::r, {.mask = byte_mask});
  bus.set_pin_drive(blofeld::McuPort::s, {.mask = s_external, .levels = 16});
  bus.write(routing, 3);
  constexpr unsigned rx_and_tx = 0x30;
  require(bus.read(port_r_input) == 0 && bus.read(port_s_input) == rx_and_tx,
          "64-pin routing moves SCI to PS5/4 and releases PR3/2");
  bus.write(s_directions, 16);
  bus.write(s_latch, 0);
  require(bus.read(port_s_input) == rx_and_tx, "SCI RX input overrides a conflicting GPIO DDR");
  bus.advance_sci(12);
  bus.set_pin_drive(blofeld::McuPort::s, {.mask = byte_mask, .levels = 16});
  reject([&bus] { static_cast<void>(bus.read(port_s_input)); },
         "A low external driver cannot silently override SCI TX high");
  bus.reset();
  require(bus.read(port_s_input) == 16,
          "Reset restores GPIO ownership and retains external drives");
}

void test_bus_receive_waveform_and_transient_contention() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus bus(flash);
  constexpr unsigned routing = 0x257;
  constexpr unsigned s_direction = 0x272;
  constexpr unsigned s_external = 0xdf;
  constexpr unsigned note_on = 0x90;
  bus.write(routing, 3);
  bus.write(baud_high, 0);
  bus.write(baud_low, midi_divider);
  bus.write(s_direction, 16);
  bus.write(control2, 4);
  const auto line = [&bus](const bool high, const unsigned cycles) {
    bus.set_pin_drive(blofeld::McuPort::s,
                      {.mask = s_external, .levels = static_cast<std::uint8_t>(high ? 16 : 0)});
    bus.advance_sci(cycles);
  };
  line(true, 3 * midi_divider);
  line(false, 16 * midi_divider);
  for (unsigned bit = 0; bit < 8; ++bit) { line(((note_on >> bit) & 1U) != 0, 16 * midi_divider); }
  line(true, 16 * midi_divider);
  require(bus.read(status1) == data_available && bus.read(data_low) == note_on,
          "A timed MIDI status waveform passes through the routed RX pad into SCIDR");
  bus.reset();
  bus.write(routing, 3);
  bus.write(baud_high, 0);
  bus.write(baud_low, 1);
  constexpr unsigned both_serial_pads = 0x30;
  bus.set_pin_drive(blofeld::McuPort::s, {.mask = byte_mask, .levels = both_serial_pads});
  bus.write(control2, 8);
  static_cast<void>(bus.read(status1));
  bus.write(data_low, 0);
  reject([&bus] { bus.advance_sci(frame_ticks * 2); },
         "Contention on intermediate low TX bits cannot vanish inside a large clock slice");
}

void test_peripheral_function_masks() {
  blofeld::McuGpio gpio;
  reject([&gpio] { gpio.set_function(blofeld::McuPort::t, {.outputs = 0x80}); },
         "Peripheral functions cannot create nonexistent pads");
  reject([&gpio] { gpio.set_function(blofeld::McuPort::s, {.inputs = 1, .outputs = 1}); },
         "Peripheral functions must select input or output");
  reject([&gpio] { gpio.set_function(blofeld::McuPort::s, {.outputs = 1, .levels = 2}); },
         "Peripheral levels must be inside their output mask");
  reject([&gpio] { static_cast<void>(gpio.read_pad(blofeld::McuPort::t, 0x80)); },
         "Pad sampling cannot create nonexistent inputs");
}
}

int main() {
  try {
    test_reset_latches_and_read_sequences();
    test_transmit_preamble_and_wire_order();
    test_back_to_back_and_disable_during_transmission();
    test_queued_preamble();
    test_receive_all_bytes_and_ninth_bit();
    test_overrun_and_framing_error_inhibition();
    test_parity_and_transmit_ninth_bit();
    test_start_and_stop_sample_tables();
    test_data_majorities_and_noise();
    test_idle_and_unknown_transfer_interval();
    test_unknown_input_and_unsupported_modes();
    test_bus_pin_routing();
    test_bus_receive_waveform_and_transient_contention();
    test_peripheral_function_masks();
    std::cout << "SCI register, serial waveform, sampling, flag and routing checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
