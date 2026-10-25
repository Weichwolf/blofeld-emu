// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "usb/usb_device.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::test::usb_device {
using namespace usb_host;
using Bytes = std::vector<std::uint8_t>;
inline constexpr std::array<std::uint8_t, 32768> descriptor_flash{};
// Own structural fixtures. Their class descriptors are not a complete USB-MIDI function.
inline constexpr std::array<std::uint8_t, 18> device{
    18, 1, 0, 2, 0, 0, 0, 8, 0x34, 0x12, 0x78, 0x56, 2, 1, 0, 0, 0, 1};
inline constexpr std::array<std::uint8_t, 55> configuration{
    9,    2,    55,   0,    2, 1, 0,    0x80, 25,   9, 4, 0, 0,    0, 1, 1,    0, 0, 5,
    0x24, 0x19, 0x37, 0xa5, 9, 4, 1,    0,    1,    1, 3, 0, 0,    7, 5, 0x81, 2, 8, 0,
    0,    9,    4,    1,    1, 1, 0xff, 0x12, 0x34, 0, 7, 5, 0x82, 2, 8, 0,    0};
inline constexpr std::array<std::uint8_t, 8> device_setup{0x80, 6, 0, 1, 0, 0, 18, 0};
inline constexpr std::array<std::uint8_t, 8> header_setup{0x80, 6, 0, 2, 0, 0, 9, 0};
inline constexpr std::array<std::uint8_t, 8> configuration_setup{0x80, 6, 0, 2, 0, 0, 55, 0};

inline bool transfer(McuBus &bus,
                     UsbDeviceProbe &probe,
                     const std::array<std::uint8_t, 8> &setup,
                     const std::span<const std::uint8_t> bytes,
                     const bool finish_status = true) {
  require(!probe.poll(bus, context) && !probe.ready(),
          "A real SETUP never completes a descriptor capture");
  constexpr unsigned setup_buffer = 0x338;
  for (unsigned i = 0; i < setup.size(); ++i) {
    require(bus.read(static_cast<std::uint16_t>(setup_buffer + i)) == setup[i],
            "Independently specified SETUP bytes reach actual MCU registers");
  }
  bus.write(status + 1, 1);
  require(!probe.poll(bus, context),
          "An empty device IN buffer produces no descriptor or progress");
  for (std::size_t offset = 0; offset < bytes.size();) {
    const auto count = std::min<std::size_t>(packet_bytes, bytes.size() - offset);
    prime(bus, UsbBuffer::control_in, bytes.subspan(offset, count));
    require(!probe.poll(bus, context) && !probe.ready(),
            "Actual IN data alone cannot complete a control transfer");
    release(bus, UsbBuffer::control_in);
    offset += count;
  }
  if (!finish_status) { return false; }
  const auto ready_now = probe.poll(bus, context);
  release(bus, UsbBuffer::control_out);
  return ready_now;
}

inline void prefix(McuBus &bus, UsbDeviceProbe &probe) {
  require(!transfer(bus, probe, device_setup, device),
          "Device transfer still needs configuration discovery");
  require(!transfer(bus, probe, header_setup, std::span(configuration).first(9)),
          "A configuration header never substitutes for the full descriptor stream");
}

inline void prepare_device(McuBus &bus) {
  prepare(bus);
  bus.write(endpoint_reset::control, endpoint_reset::mce_high);
  constexpr unsigned counts = 0x21;       // One configuration with two interfaces.
  constexpr unsigned alternatives = 0x21; // Interface0: one alt; interface1: two.
  endpoint_reset::configure(bus, {.bank = 8, .value = counts});
  endpoint_reset::configure(bus, {.bank = 9, .value = alternatives});
  bus.write(endpoint_reset::control, endpoint_reset::men_high);
}

inline UsbDeviceDescriptors capture(McuBus &bus) {
  UsbDeviceProbe probe;
  prefix(bus, probe);
  require(transfer(bus, probe, configuration_setup, configuration),
          "All three actual control transfers complete");
  return probe.descriptors();
}

inline UsbDeviceDescriptors capture() {
  McuBus bus(descriptor_flash);
  prepare_device(bus);
  return capture(bus);
}
}
