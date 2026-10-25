// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "usb/usb_device.hpp"
#include "usb_device_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_device;

void test_complete_capture_and_status_replay() {
  McuBus bus(descriptor_flash);
  prepare(bus);
  UsbDeviceProbe probe;
  reject([&probe] { static_cast<void>(probe.descriptors()); }, "unavailable");
  prefix(bus, probe);
  require(!transfer(bus, probe, configuration_setup, configuration, false),
          "Last IN transfer is still awaiting its real status handshake");
  reject([&probe] { static_cast<void>(probe.descriptors()); }, "unavailable");
  const auto saved_bus = bus.clone();
  const auto saved_probe = probe;
  require(probe.poll(bus, context), "The actual OUT status handshake completes capture");
  require(std::ranges::equal(probe.descriptors().device(), device) &&
              std::ranges::equal(probe.descriptors().configuration(), configuration),
          "Immutable metadata retains every actual descriptor byte, including opaque class data");
  constexpr std::array<UsbInterfaceDescription, 3> expected{
      UsbInterfaceDescription{.number = 0, .alternate = 0, .device_class = 1, .subclass = 1},
      UsbInterfaceDescription{.number = 1, .alternate = 0, .device_class = 1, .subclass = 3},
      UsbInterfaceDescription{
          .number = 1, .alternate = 1, .device_class = 0xff, .subclass = 0x12, .protocol = 0x34}};
  require(
      std::ranges::equal(probe.descriptors().alternatives(), expected) &&
          probe.descriptors().interface_count() == 2,
      "Independent interface alternatives are preserved without choosing an active alternative");
  auto replay_bus = saved_bus->clone();
  auto replay_probe = saved_probe;
  require(
      replay_probe.poll(*replay_bus, context) &&
          std::ranges::equal(replay_probe.descriptors().configuration(), configuration),
      "Joint capture/controller replay retains the pending status, not a fabricated completion");
  probe.reset();
  require(!probe.ready() && !probe.faulted(), "Host reset discards all captured identity");
  reject([&probe] { static_cast<void>(probe.descriptors()); }, "unavailable");
}

void malformed_configuration(const Bytes &bytes, const std::string_view expected) {
  McuBus bus(descriptor_flash);
  prepare(bus);
  UsbDeviceProbe probe;
  prefix(bus, probe);
  reject([&] { static_cast<void>(transfer(bus, probe, configuration_setup, bytes)); }, expected);
  require(probe.faulted() && !probe.ready(),
          "Invalid descriptors poison the probe without exportable identity");
  reject([&] { static_cast<void>(probe.poll(bus, context)); }, "Faulted USB descriptor probe");
}

void test_independent_descriptor_failures() {
  enum Offset : std::uint8_t {
    class_length = 18,
    interface_type = 24,
    interface_number = 25,
    first_alternate = 26,
    endpoint_count = 27,
    endpoint_address = 34,
    second_alternate = 42
  };

  constexpr unsigned changed_maximum_power = 26;
  auto bytes = Bytes(configuration.begin(), configuration.end());
  bytes[8] = changed_maximum_power;
  malformed_configuration(bytes, "changed between header");
  bytes.assign(configuration.begin(), configuration.end());
  bytes[class_length] = 0;
  malformed_configuration(bytes, "zero-length");
  bytes.assign(configuration.begin(), configuration.end());
  bytes[interface_number] = 0;
  malformed_configuration(bytes, "duplicate USB interface");
  bytes.assign(configuration.begin(), configuration.end());
  bytes[endpoint_address] = 0x80;
  malformed_configuration(bytes, "endpoint descriptor");
  bytes.assign(configuration.begin(), configuration.end());
  bytes[endpoint_count] = 2;
  malformed_configuration(bytes, "incomplete preceding endpoint");
  bytes.assign(configuration.begin(), configuration.end());
  bytes[interface_type] = 2;
  malformed_configuration(bytes, "nested USB");
  bytes.assign(configuration.begin(), configuration.end());
  bytes[first_alternate] = 1;
  bytes[second_alternate] = 2;
  malformed_configuration(bytes, "default interface");
  bytes.assign(configuration.begin(), configuration.end());
  bytes.pop_back();
  malformed_configuration(bytes, "complete successful control transfer");
}

void test_endpoint_ownership_between_interfaces() {
  constexpr std::array<std::uint8_t, 41> duplicate_endpoint{
      9, 2, 41, 0, 2, 1, 0, 0x80, 1, 9,    4, 0, 0, 1, 0xff, 0,    0, 0, 7, 5, 0x81,
      2, 8, 0,  0, 9, 4, 1, 0,    1, 0xff, 0, 0, 0, 7, 5,    0x81, 2, 8, 0, 0};
  constexpr std::array<std::uint8_t, 8> setup{0x80, 6, 0, 2, 0, 0, 41, 0};
  McuBus bus(descriptor_flash);
  prepare(bus);
  UsbDeviceProbe probe;
  static_cast<void>(transfer(bus, probe, device_setup, device));
  static_cast<void>(transfer(bus, probe, header_setup, std::span(duplicate_endpoint).first(9)));
  reject([&] { static_cast<void>(transfer(bus, probe, setup, duplicate_endpoint)); },
         "endpoint descriptor");
}

void test_descriptor_resource_boundary() {
  constexpr unsigned opaque_count_at_limit = 1022;
  constexpr std::array<std::uint8_t, 2> opaque{2, 0xff};
  for (const bool exceeds_limit : {false, true}) {
    constexpr std::array<std::uint8_t, 18> own_prefix{
        9, 2, 0, 0, 1, 1, 0, 0x80, 1, 9, 4, 0, 0, 0, 0xff, 0, 0, 0};
    Bytes bytes(own_prefix.begin(), own_prefix.end());
    for (unsigned i = 0; i < opaque_count_at_limit + (exceeds_limit ? 1U : 0U); ++i) {
      bytes.insert(bytes.end(), opaque.begin(), opaque.end());
    }
    bytes[2] = static_cast<std::uint8_t>(exceeds_limit ? 16 : 14);
    bytes[3] = 8;
    const std::array<std::uint8_t, 8> setup{0x80, 6, 0, 2, 0, 0, bytes[2], 8};
    McuBus bus(descriptor_flash);
    prepare(bus);
    UsbDeviceProbe probe;
    static_cast<void>(transfer(bus, probe, device_setup, device));
    static_cast<void>(transfer(bus, probe, header_setup, std::span(bytes).first(9)));
    if (exceeds_limit) {
      reject([&] { static_cast<void>(transfer(bus, probe, setup, bytes)); }, "excessive");
    } else {
      require(transfer(bus, probe, setup, bytes) && probe.descriptors().alternatives().size() == 1,
              "Exactly1024 independently constructed descriptors retain one real interface");
    }
  }
}

void test_device_and_header_failures() {
  for (const auto change : {std::array<unsigned, 2>{0, 17}, {7, 16}, {17, 2}, {3, 3}}) {
    McuBus bus(descriptor_flash);
    prepare(bus);
    UsbDeviceProbe probe;
    auto bytes = device;
    bytes.at(change[0]) = static_cast<std::uint8_t>(change[1]);
    reject([&] { static_cast<void>(transfer(bus, probe, device_setup, bytes)); }, "USB");
    require(probe.faulted() && !probe.ready(),
            "Invalid device metadata cannot progress to configuration");
  }
  for (const auto change : {std::array<unsigned, 2>{0, 8}, {2, 8}, {4, 33}, {5, 0}}) {
    McuBus bus(descriptor_flash);
    prepare(bus);
    UsbDeviceProbe probe;
    static_cast<void>(transfer(bus, probe, device_setup, device));
    auto bytes = configuration;
    bytes.at(change[0]) = static_cast<std::uint8_t>(change[1]);
    reject(
        [&] { static_cast<void>(transfer(bus, probe, header_setup, std::span(bytes).first(9))); },
        "USB");
    require(probe.faulted(), "Malformed headers fail before another transfer can be requested");
  }
  McuBus bus(descriptor_flash);
  UsbDeviceProbe probe;
  reject([&] { static_cast<void>(probe.poll(bus, context)); }, "mcu-pc=0x8123 ccr=0x42");
  require(probe.faulted(), "An unready controller is never made ready by discovery");
}
}

int main() {
  try {
    test_complete_capture_and_status_replay();
    test_independent_descriptor_failures();
    test_device_and_header_failures();
    test_descriptor_resource_boundary();
    test_endpoint_ownership_between_interfaces();
    std::cout << "Verified MCU-backed descriptor capture, actual control status, replay and strict "
                 "malformed/unknown boundaries. No original enumeration claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
