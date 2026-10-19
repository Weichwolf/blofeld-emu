// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_usb.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test {
namespace {
constexpr unsigned sci_baud_high = 0xc8;
constexpr unsigned sci_baud_low = 0xc9;
constexpr unsigned sci_control = 0xcb;
constexpr unsigned sci_status = 0xcc;
constexpr unsigned sci_data = 0xcf;
constexpr unsigned rti_control = 0x3b;
constexpr unsigned rti_period = 8192;
constexpr unsigned rti_rate = 0x40;
constexpr unsigned irq_edge_enabled = 0xc0;
constexpr unsigned receive_pin = 4;
constexpr unsigned pattern = 0x55;
constexpr unsigned receive_ready = 0xe0;
constexpr unsigned sci_idle = 0xc0;
constexpr unsigned usb_selector = 0x30e;
constexpr unsigned usb_data = 0x310;
constexpr unsigned usb_buffer = 0x380;
constexpr unsigned usb_endpoint2 = 0x318;
constexpr unsigned usb_valid = 0x50;
constexpr unsigned usb_complete = 0x30;
constexpr unsigned timer_counter = 0x44;
constexpr unsigned timer_control1 = 0x46;
constexpr unsigned timer_edges = 0x4b;
constexpr unsigned timer_control2 = 0x4d;
constexpr unsigned timer_flags = 0x4e;
constexpr unsigned timer_capture = 0x50;
constexpr unsigned timer_modes = 0x40;
constexpr unsigned timer_channel7 = 0x5e;
constexpr unsigned usb_control = 0x300;
constexpr unsigned usb_configuration_status = 0x30c;
constexpr unsigned usb_buffers = 0x30f;

void test_gpio_clock_and_edge_history() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  drive_interrupts(machine.bus());
  machine.bus().write(hc12::irq_control, irq_edge_enabled);
  machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
  drive_interrupts(machine.bus());
  machine.bus().write(rti_control, rti_rate);
  machine.bus().advance_oscillator(rti_period - 3);
  machine.bus().write(hc12::ddra, byte_mask);
  const auto checkpoint = machine.checkpoint();
  machine.bus().acknowledge_interrupt(irq_vector);
  machine.bus().write(hc12::irq_control, 0);
  machine.bus().advance_oscillator(3);
  machine.bus().write(hc12::ddrq, byte_mask);
  machine.bus().write(hc12::ptq, pattern);
  machine.bus().write(hc12::porta, pattern);
  machine.restore(checkpoint);
  const auto pending = machine.bus().interrupt_request(irq_edge_enabled);
  require(machine.bus().oscillator_cycles_until_rti() == 3 && pending &&
              pending->vector == irq_vector,
          "Snapshot preserves fractional RTI phase and an IRQ edge after its pin returned high");
  machine.bus().acknowledge_interrupt(irq_vector);
  require(!machine.bus().interrupt_request(irq_edge_enabled),
          "Acknowledging a restored edge consumes it once");
  machine.bus().write(hc12::irq_control, rti_rate);
  require(machine.bus().read(hc12::irq_control) == irq_edge_enabled,
          "Snapshot preserves the write-once edge configuration latch");
  machine.bus().write(hc12::ddrq, byte_mask);
  require(machine.bus().read(hc12::ptq) == 0, "Checkpoint retains the known PIM reset latch");
  reject([&machine] { static_cast<void>(machine.bus().read(hc12::porta)); },
         "Cloning does not initialize unknown GPIO output latches");
  machine.bus().write(hc12::ddrq, 0);
  machine.bus().set_pin_drive(McuPort::q, {.floating = byte_mask});
  machine.bus().write(hc12::perq, byte_mask);
  const auto floating = machine.checkpoint();
  machine.bus().set_pin_drive(McuPort::q, {.mask = byte_mask, .levels = 0});
  machine.restore(floating);
  require(machine.bus().read(hc12::ptq) == byte_mask,
          "Snapshot distinguishes explicitly floating external pads from driven low pads");
  machine.bus().set_pin_drive(McuPort::e, {.mask = 1, .levels = 1});
  const auto unknown_edge = machine.checkpoint();
  machine.bus().acknowledge_interrupt(irq_vector);
  drive_interrupts(machine.bus());
  machine.restore(unknown_edge);
  reject([&machine] { static_cast<void>(machine.bus().interrupt_request(irq_edge_enabled)); },
         "Restoring an unknown edge history cannot fabricate an absent IRQ");
}

void receive_level(McuBus &bus, const bool high, const unsigned samples = 16) {
  bus.set_pin_drive(
      McuPort::r,
      {.mask = receive_pin, .levels = static_cast<std::uint8_t>(high ? receive_pin : 0)});
  bus.advance_sci(samples);
}

void finish_receive(McuMachine &machine) {
  for (unsigned bit = 4; bit < 8; ++bit) {
    receive_level(machine.bus(), ((pattern >> bit) & 1U) != 0);
  }
  receive_level(machine.bus(), true);
  require(machine.bus().read(sci_status) == receive_ready,
          "Restored half frame reaches RDRF at the independently expected stop boundary");
  const auto acknowledged = machine.checkpoint();
  machine.bus().read(sci_data);
  machine.restore(acknowledged);
  require(machine.bus().read(sci_data) == pattern && machine.bus().read(sci_status) == sci_idle,
          "Snapshot preserves received bits and the status-read-before-data acknowledgment latch");
}

void test_sci_partial_frame_and_prepared_transmit() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  machine.bus().write(sci_baud_high, 0);
  machine.bus().write(sci_baud_low, 1);
  machine.bus().write(sci_control, 4);
  receive_level(machine.bus(), true);
  receive_level(machine.bus(), false);
  for (unsigned bit = 0; bit < 4; ++bit) {
    receive_level(machine.bus(), ((pattern >> bit) & 1U) != 0);
  }
  const auto half_frame = machine.checkpoint();
  finish_receive(machine);
  machine.restore(half_frame);
  finish_receive(machine);
  machine.bus().write(sci_control, 8);
  static_cast<void>(machine.bus().read(sci_status));
  machine.bus().write(sci_data, pattern);
  constexpr unsigned before_start = 159;
  machine.bus().advance_sci(before_start);
  const auto prepared = machine.checkpoint();
  machine.bus().advance_sci(before_start);
  machine.restore(prepared);
  machine.bus().advance_sci(1);
  require((machine.bus().observe_pads(McuPort::r, 8).levels & 8U) == 0,
          "Prepared transmitter restored one sample before start drives a real start bit");
  for (unsigned bit = 0; bit < 8; ++bit) {
    machine.bus().advance_sci(16);
    const auto level = (machine.bus().observe_pads(McuPort::r, 8).levels & 8U) != 0;
    require(level == (((pattern >> bit) & 1U) != 0),
            "Prepared TX state retains the literal LSB-first 55 waveform");
  }
  machine.bus().advance_sci(16 * 2);
  require(machine.bus().read(sci_status) == sci_idle, "Restored transmission completes normally");
  machine.bus().write(sci_baud_high, 1);
  const auto pending_baud = machine.checkpoint();
  machine.bus().write(sci_baud_low, 2);
  machine.restore(pending_baud);
  reject([&machine] { static_cast<void>(machine.bus().read(sci_baud_high)); },
         "Snapshot retains an uncommitted baud high byte without fabricated readback");
  machine.bus().write(sci_baud_low, 3);
  require(machine.bus().read(sci_baud_high) == 1 && machine.bus().read(sci_baud_low) == 3,
          "Original staged baud high byte commits with the new low byte");
}

void test_timer_phase_and_capture() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  machine.bus().set_pin_drive(McuPort::t, {.mask = 1, .levels = 0});
  machine.bus().sample_timer_inputs(1);
  machine.bus().write(timer_edges, 1);    // Rising capture on channel 0.
  machine.bus().write(timer_control2, 2); // Divide by four.
  machine.bus().write(timer_control1, 0x80);
  machine.bus().advance_timer(3);
  const auto partial = machine.checkpoint();
  machine.bus().advance_timer(1);
  machine.restore(partial);
  require(machine.bus().read_memory({.address = timer_counter, .size = 2}) == 0,
          "Timer snapshot retains sub-prescaler phase without an early count");
  machine.bus().advance_timer(1);
  machine.bus().set_pin_drive(McuPort::t, {.mask = 1, .levels = 1});
  machine.bus().sample_timer_inputs(1);
  require(machine.bus().read_memory({.address = timer_capture, .size = 2}) == 1 &&
              machine.bus().read(timer_flags) == 1,
          "Saved input history and exact prescaler phase produce the expected rising capture");
}

void test_timer_pending_reset_and_divisor_change() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  constexpr unsigned channel7_mask = 0x80;
  machine.bus().write(timer_modes, channel7_mask);
  machine.bus().write_memory({.address = timer_channel7, .size = 2, .value = 2});
  machine.bus().write(timer_control2, 8);
  machine.bus().write(timer_control1, channel7_mask);
  machine.bus().advance_timer(2);
  const auto reset_pending = machine.checkpoint();
  machine.bus().advance_timer(2);
  machine.restore(reset_pending);
  require(machine.bus().read_memory({.address = timer_counter, .size = 2}) == 2 &&
              machine.bus().read(timer_flags) == channel7_mask,
          "Snapshot captures channel-7 match before its deferred counter reset");
  machine.bus().advance_timer(1);
  require(machine.bus().read_memory({.address = timer_counter, .size = 2}) == 0,
          "A restored pending reset consumes its own bus cycle without an extra count");
  machine.bus().advance_timer(1);
  require(machine.bus().read_memory({.address = timer_counter, .size = 2}) == 1,
          "Counting resumes one cycle after the restored reset");
  machine.bus().reset();
  machine.bus().write(timer_control1, channel7_mask);
  machine.bus().advance_timer(3);
  machine.bus().write(timer_control2, 2);
  constexpr unsigned before_synchronization = 124;
  machine.bus().advance_timer(before_synchronization);
  const auto divisor_pending = machine.checkpoint();
  machine.bus().advance_timer(16);
  machine.restore(divisor_pending);
  machine.bus().advance_timer(4);
  require(machine.bus().read_memory({.address = timer_counter, .size = 2}) == 128,
          "Snapshot distinguishes requested divide-by-four from the still active divisor");
  machine.bus().advance_timer(1);
  constexpr unsigned after_synchronization = 129;
  require(machine.bus().read_memory({.address = timer_counter, .size = 2}) == after_synchronization,
          "Restored prescaler changes at the original synchronization boundary");
}

struct Configuration {
  unsigned selection = 0;
  std::uint32_t value = 0;
};

void usb_configuration(McuBus &bus, const Configuration configuration) {
  bus.write(usb_selector, static_cast<std::uint8_t>(configuration.selection));
  bus.write_memory({.address = usb_data, .size = 2, .value = configuration.value >> 16U});
  bus.write_memory({.address = usb_data + 2, .size = 2, .value = configuration.value & word_mask});
}

void test_usb_in_flight_and_staged_words() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  machine.bus().write(usb_control, rti_rate);
  machine.bus().write(usb_control + 1, 8);
  machine.bus().observe_phy({.edges = 15});
  constexpr unsigned default_endpoint = 0x02000000;
  constexpr unsigned bulk_input = 0x020000d1;
  constexpr unsigned configurations = 0x11;
  usb_configuration(machine.bus(), {.selection = 1, .value = default_endpoint});
  usb_configuration(machine.bus(), {.selection = 2, .value = bulk_input});
  usb_configuration(machine.bus(), {.selection = 8, .value = configurations});
  machine.bus().observe_usb_decoder({});
  machine.bus().write(usb_control, sci_idle);
  machine.bus().observe_usb_decoder({.enumerated = true});
  machine.bus().observe_usb_configuration({.configuration = 1});
  machine.bus().write(usb_configuration_status, 0x10);
  machine.bus().write(usb_buffers, 2);
  const auto unknown_buffer = machine.checkpoint();
  machine.bus().write(usb_buffer, pattern);
  machine.bus().write(usb_endpoint2 + 1, 1);
  machine.bus().write(usb_endpoint2, usb_valid);
  require(machine.bus().request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::data,
          "Own USB input packet begins before the checkpoint");
  const auto in_flight = machine.checkpoint();
  machine.bus().finish_usb_in(true);
  machine.restore(in_flight);
  reject([&machine] { machine.bus().write(usb_buffer, 0); },
         "Restored in-flight packet preserves USB ownership");
  machine.bus().finish_usb_in(false);
  const auto lost_ack = machine.checkpoint();
  const auto retry = machine.bus().request_usb_in(UsbBuffer::endpoint2);
  require(retry.size == 1 && retry.data[0] == pattern && !retry.data1,
          "Restored retry returns the original payload and DATA0 after lost acknowledgment");
  machine.bus().finish_usb_in(true);
  machine.restore(lost_ack);
  reject([&machine] { machine.bus().finish_usb_in(true); },
         "Lost-ACK snapshot must not retain a closed handshake window");
  const auto retried = machine.bus().request_usb_in(UsbBuffer::endpoint2);
  require(!retried.data1 && retried.data[0] == pattern,
          "Cached transmitted data survives snapshot independently of pending handshake state");
  machine.bus().finish_usb_in(true);
  machine.bus().write(usb_endpoint2, usb_complete);
  machine.bus().write(usb_endpoint2, usb_valid);
  require(machine.bus().request_usb_in(UsbBuffer::endpoint2).data1,
          "Acknowledgment after restore toggles the next packet to DATA1 exactly once");
  machine.bus().finish_usb_in(true);
  machine.bus().write(usb_selector, 5);
  constexpr unsigned high_word = 0x1234;
  constexpr unsigned low_word = 0x5678;
  machine.bus().write_memory({.address = usb_data, .size = 2, .value = high_word});
  const auto staged = machine.checkpoint();
  machine.bus().write_memory({.address = usb_data + 2, .size = 2, .value = 0});
  machine.restore(staged);
  machine.bus().write_memory({.address = usb_data + 2, .size = 2, .value = low_word});
  require(machine.bus().read_memory({.address = usb_data, .size = 2}) == high_word &&
              machine.bus().read_memory({.address = usb_data + 2, .size = 2}) == low_word,
          "Snapshot preserves the uncommitted USB configuration high word");
  machine.restore(unknown_buffer);
  reject([&machine] { static_cast<void>(machine.bus().read(usb_buffer)); },
         "Restoring an unknown USB buffer must not reveal later branch payload bytes");
}
}

void test_checkpoint_peripherals() {
  test_gpio_clock_and_edge_history();
  test_sci_partial_frame_and_prepared_transmit();
  test_timer_phase_and_capture();
  test_timer_pending_reset_and_divisor_change();
  test_usb_in_flight_and_staged_words();
}
}
