// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "mcu/mcu_usb_registers.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {

std::optional<std::uint8_t>
McuUsb::decode_address_setup(const std::array<std::uint8_t, 8> &data) const {
  constexpr unsigned request_type = 0x60;
  constexpr unsigned set_address = 5;
  constexpr unsigned maximum_address = 127;
  if ((data[0] & request_type) != 0 || data[1] != set_address) { return std::nullopt; }
  // USB2.0 9.4.6 leaves zero/out-of-range/malformed/configured cases unspecified.
  if (data[0] != 0 || data[2] == 0 || data[2] > maximum_address || data[3] != 0 || data[4] != 0 ||
      data[5] != 0 || data[6] != 0 || data[7] != 0) {
    throw Fault("Unqualified automatic USB SET_ADDRESS value/recipient/index/length");
  }
  if ((configuration_status_ & (usb::configuration_request | (usb::nibble_mask << 8U))) != 0 ||
      (status_ & (usb::setup | usb::setup_overrun)) != 0) {
    throw Fault(
        "Unqualified USB SET_ADDRESS while configured or firmware SETUP/configuration pending");
  }
  return data[2];
}

std::optional<McuUsb::AutomaticRead>
McuUsb::decode_read_setup(const std::array<std::uint8_t, 8> &data) const {
  constexpr unsigned request_type = 0x60;
  constexpr unsigned get_configuration = 8;
  constexpr unsigned get_interface = 10;
  constexpr unsigned device_input = 0x80;
  constexpr unsigned interface_input = 0x81;
  if ((data[0] & request_type) != 0 || (data[1] != get_configuration && data[1] != get_interface)) {
    return std::nullopt;
  }
  // USB2.0 9.4.2/9.4.4 leave malformed value/length behavior unspecified.
  if (data[2] != 0 || data[3] != 0 || data[5] != 0 || data[6] != 1 || data[7] != 0) {
    throw Fault("Unqualified automatic USB read SETUP value/index/length");
  }
  if ((configuration_status_ & usb::configuration_request) != 0 ||
      (status_ & (usb::setup | usb::setup_overrun)) != 0) {
    throw Fault("Unqualified automatic USB read overlapping pending firmware SETUP/configuration");
  }
  if (data[1] == get_configuration) {
    if (data[0] != device_input || data[4] != 0) {
      throw Fault("Unqualified USB GET_CONFIGURATION direction/recipient/index");
    }
    const auto configuration = (configuration_status_ >> 8U) & usb::nibble_mask;
    if (configuration > 2 ||
        (configuration != 0 && (configuration_status_ & usb::configuration_valid) == 0)) {
      throw Fault("Unqualified USB GET_CONFIGURATION current selection");
    }
    if (configuration == 0 && device_address_ == 0) {
      throw Fault("Unqualified USB GET_CONFIGURATION in default/unaddressed state");
    }
    return AutomaticRead{.value = static_cast<std::uint8_t>(configuration)};
  }
  if (data[0] != interface_input) {
    throw Fault("Unqualified USB GET_INTERFACE direction/recipient");
  }
  const auto configuration = host_configuration();
  if (configuration.configuration == 0 && device_address_ == 0) {
    throw Fault("Unqualified USB GET_INTERFACE in default/unaddressed state");
  }
  if (configuration.configuration != 0 && data[4] == configuration.interfaces &&
      configuration.interfaces < configuration.alternatives.size()) {
    throw Fault("Unqualified USB GET_INTERFACE interface-count boundary");
  }
  if (configuration.configuration == 0 || data[4] >= configuration.interfaces) {
    return AutomaticRead{.error = true};
  }
  return AutomaticRead{.value = configuration.alternatives.at(data[4])};
}

std::optional<UsbConfigurationRequest>
McuUsb::decode_configuration_setup(const std::array<std::uint8_t, 8> &data) const {
  constexpr unsigned set_configuration = 9;
  constexpr unsigned set_interface = 11;
  constexpr unsigned request_type = 0x60;
  if ((data[0] & request_type) != 0 || (data[1] != set_configuration && data[1] != set_interface)) {
    return std::nullopt;
  }
  if (data[3] != 0 || data[5] != 0 || data[6] != 0 || data[7] != 0) {
    throw Fault("Unsupported USB configuration SETUP value/index/length");
  }
  if (data[1] == set_configuration) {
    if (data[0] != 0 || data[4] != 0) {
      throw Fault("Unsupported USB SET_CONFIGURATION direction/recipient/index");
    }
    return UsbConfigurationRequest{.configuration = data[2]};
  }
  if (data[0] != 1 || (configuration_status_ & usb::configuration_valid) == 0 ||
      (configuration_status_ & (usb::nibble_mask << 8U)) == 0) {
    throw Fault("Unsupported USB SET_INTERFACE direction/recipient/unconfigured state");
  }
  return UsbConfigurationRequest{
      .configuration = static_cast<std::uint8_t>((configuration_status_ >> 8U) & usb::nibble_mask),
      .interface = data[4],
      .alternate = data[2]};
}

}
