// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "usb/usb_midi_host.hpp"
#include "usb/usb_transfer.hpp"
#include "usb_host_fixture.hpp"
#include "usb_midi_host_fixture.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace {
using namespace blofeld;
namespace fixture = test::usb_midi_host;
using test::usb_host::reject;
using test::usb_host::require;

void native_enumeration_and_bidirectional_packets() {
  fixture::Fixture device;
  reject([&] { static_cast<void>(device.host.send_packets(fixture::note)); }, "configured");
  device.enumerate();
  require(std::ranges::equal(device.host.descriptors().configuration(), fixture::configuration) &&
              device.bus.usb_host_address() == 1 &&
              device.bus.usb_host_configuration().configuration == 1,
          "Native ROM descriptors, actual SET_ADDRESS and SET_CONFIGURATION establish readiness");
  reject([&] { static_cast<void>(device.host.send_packets(std::span(fixture::note).first(3))); },
         "complete events");
  const auto sequence = device.host.send_packets(fixture::note);
  const auto sent = device.host.poll(device.bus, {}, device.tick);
  require(sent.size() == 1 && sent.front().sequence == sequence && sent.front().transferred == 4 &&
              !sent.front().input_direction,
          "Only actual USB OUT ACK completes the original four-byte host packet");
  require(device.host.poll(device.bus, {}, device.tick).empty(),
          "No duplicate packet completes while the actual native OUT handler is pending");
  device.service();
  constexpr unsigned first_word = 0x0990;
  constexpr unsigned second_word = 0x3c64;
  require(test::usb_host::word(device.bus, fixture::echo) == first_word &&
              test::usb_host::word(device.bus, fixture::echo + 2) == second_word,
          "Own native bulk OUT ISR reads and records independently expected MIDI bytes");
  const auto received = device.host.poll(device.bus, {}, device.tick);
  device.service();
  require(received.size() == 1 && received.front().input_direction &&
              received.front().input == std::vector(fixture::note.begin(), fixture::note.end()) &&
              !device.cpu.pending_interrupt(),
          "Own native USB RAM stores produce the complete IN echo and actual completion IRQ");
}

void configuration_requires_firmware_acknowledgement() {
  fixture::Fixture device;
  for (unsigned poll = 0; poll < fixture::maximum_polls; ++poll) {
    static_cast<void>(device.poll(false));
    const auto request = device.cpu.pending_interrupt();
    if (request && request->vector == fixture::config_vector) {
      require(!device.host.ready(), "Configuration SETUP alone cannot establish readiness");
      static_cast<void>(device.poll(false));
      require(!device.host.ready(), "NAK status retains the actual pending configuration");
      reject([&] { static_cast<void>(device.bus.usb_host_configuration()); }, "pending");
      device.service();
      device.enumerate();
      return;
    }
    device.service();
  }
  throw Fault("Own configuration request did not reach its native ISR");
}

void bounded_output_and_reset_retirement() {
  fixture::Fixture device;
  device.enumerate();
  static_cast<void>(device.poll()); // Reserve the one pending IN transfer.
  for (unsigned index = 1; index < maximum_usb_pending; ++index) {
    static_cast<void>(device.host.send_packets(fixture::note));
  }
  reject([&] { static_cast<void>(device.host.send_packets(fixture::note)); }, "full");
  device.bus.observe_usb_decoder({.reset = true});
  device.bus.observe_usb_decoder({.enumerated = true});
  reject([&] { static_cast<void>(device.poll()); }, "lost its link");
  require(!device.host.ready(), "A reset cannot reuse an old MIDI host configuration");
  reject([&] { static_cast<void>(device.host.send_packets(fixture::note)); }, "configured");
}

void enumeration_timeout() {
  fixture::Fixture device;
  UsbMidiHost host(1, 2);
  require(host.poll(device.bus, {}, 0).empty(), "A descriptor request alone supplies no response");
  reject([&] { static_cast<void>(host.poll(device.bus, {}, 2)); }, "timeout");
  require(!host.ready(), "An expired descriptor request cannot report MIDI readiness");
}
}

int main() {
  try {
    native_enumeration_and_bidirectional_packets();
    configuration_requires_firmware_acknowledgement();
    bounded_output_and_reset_retirement();
    enumeration_timeout();
    std::cout
        << "Native USB MIDI enumeration, bidirectional packets and failure contracts passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
