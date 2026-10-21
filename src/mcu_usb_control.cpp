// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "mcu_usb_registers.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {

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
