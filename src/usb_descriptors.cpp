// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/usb_device.hpp"
#include "usb_descriptor_detail.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace blofeld::usb_descriptor {
std::uint16_t little_word(const std::span<const std::uint8_t> bytes, const std::size_t offset) {
  if (offset >= bytes.size() || bytes.size() - offset < 2) {
    throw Fault("Truncated USB descriptor word");
  }
  return static_cast<std::uint16_t>(static_cast<unsigned>(bytes[offset]) |
                                    (static_cast<unsigned>(bytes[offset + 1]) << 8U));
}

void validate_device(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() != device_bytes || bytes[0] != device_bytes || bytes[1] != device_type) {
    throw Fault("Invalid USB device descriptor length/type");
  }
  constexpr unsigned usb_1_0 = 0x0100;
  constexpr unsigned usb_1_1 = 0x0110;
  constexpr unsigned usb_2_0 = 0x0200;
  const auto revision = little_word(bytes, 2);
  if (revision != usb_1_0 && revision != usb_1_1 && revision != usb_2_0) {
    throw Fault("Unsupported USB device specification revision");
  }
  constexpr std::array<unsigned, 4> full_speed_packets{8, 16, 32, 64};
  bool packet_valid = false;
  for (const auto size : full_speed_packets) { packet_valid = packet_valid || bytes[7] == size; }
  constexpr unsigned configuration_count_offset = 17;
  if (!packet_valid || bytes[configuration_count_offset] != 1) {
    throw Fault("Unsupported USB default packet size or multiple/absent configurations");
  }
}

unsigned configuration_length(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() < configuration_header_bytes || bytes[0] != configuration_header_bytes ||
      bytes[1] != configuration_type) {
    throw Fault("Invalid USB configuration descriptor header");
  }
  const auto length = little_word(bytes, 2);
  if (length < configuration_header_bytes || bytes[4] == 0 || bytes[4] > maximum_interfaces ||
      bytes[5] == 0) {
    throw Fault("Unsupported USB configuration length/interface count/value");
  }
  return length;
}

namespace {
struct EndpointList {
  unsigned expected = 0;
  unsigned received = 0;
  unsigned owner = 0;
  std::bitset<256> seen;
  std::array<unsigned, 256> interfaces{};

  void begin(const std::span<const std::uint8_t> descriptor) {
    if (received != expected) {
      throw Fault("Invalid USB interface: incomplete preceding endpoint list");
    }
    owner = static_cast<unsigned>(descriptor[2]) + 1;
    expected = descriptor[4];
    received = 0;
    seen.reset();
  }

  void accept(const std::span<const std::uint8_t> descriptor) {
    constexpr unsigned endpoint_bytes = 7;
    constexpr unsigned reserved_address = 0x70;
    if (descriptor.size() < endpoint_bytes || owner == 0 || (descriptor[2] & 15U) == 0 ||
        (descriptor[2] & reserved_address) != 0 || seen.test(descriptor[2]) ||
        (interfaces[descriptor[2]] != 0 && interfaces[descriptor[2]] != owner) ||
        ++received > expected) {
      throw Fault("Invalid/duplicate USB endpoint descriptor");
    }
    seen.set(descriptor[2]);
    interfaces[descriptor[2]] = owner;
  }
};
}

std::vector<UsbInterfaceDescription>
parse_configuration(const std::span<const std::uint8_t> bytes) {
  if (configuration_length(bytes) != bytes.size()) {
    throw Fault("USB configuration total length mismatch");
  }
  constexpr unsigned interface_type = 4;
  constexpr unsigned endpoint_type = 5;
  constexpr unsigned interface_bytes = 9;
  std::array<std::bitset<256>, maximum_interfaces> alternatives;
  EndpointList endpoints;
  std::vector<UsbInterfaceDescription> result;
  unsigned count = 1;
  for (std::size_t offset = configuration_header_bytes; offset < bytes.size();) {
    if (bytes.size() - offset < 2 || bytes[offset] < 2 || bytes[offset] > bytes.size() - offset ||
        ++count > maximum_descriptors) {
      throw Fault("Truncated, zero-length or excessive USB subordinate descriptors");
    }
    const auto descriptor = bytes.subspan(offset, bytes[offset]);
    if (descriptor[1] == interface_type) {
      if (descriptor.size() != interface_bytes || descriptor[2] >= bytes[4] ||
          alternatives.at(descriptor[2]).test(descriptor[3])) {
        throw Fault("Invalid/duplicate USB interface or incomplete preceding endpoint list");
      }
      alternatives.at(descriptor[2]).set(descriptor[3]);
      endpoints.begin(descriptor);
      result.push_back({.number = descriptor[2],
                        .alternate = descriptor[3],
                        .device_class = descriptor[5],
                        .subclass = descriptor[6],
                        .protocol = descriptor[7]});
    } else if (descriptor[1] == endpoint_type) {
      endpoints.accept(descriptor);
    } else if (descriptor[1] == device_type || descriptor[1] == configuration_type) {
      throw Fault("Unexpected nested USB device/configuration descriptor");
    }
    offset += descriptor.size();
  }
  if (endpoints.received != endpoints.expected) { throw Fault("Incomplete USB endpoint list"); }
  for (unsigned number = 0; number < bytes[4]; ++number) {
    if (!alternatives.at(number).test(0)) {
      throw Fault("Missing USB default interface alternative");
    }
  }
  return result;
}
}

namespace blofeld {
UsbDeviceDescriptors::UsbDeviceDescriptors(std::vector<std::uint8_t> device,
                                           std::vector<std::uint8_t> configuration)
    : device_(std::move(device)),
      configuration_(std::move(configuration)),
      alternatives_(usb_descriptor::parse_configuration(configuration_)) {
  usb_descriptor::validate_device(device_);
}
}
