// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "mcu/mcu_usb_registers.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
std::optional<McuUsb::AutomaticRead>
McuUsb::decode_status_setup(const std::array<std::uint8_t, 8> &data) const {
  constexpr unsigned type = 0x60;
  constexpr unsigned device_input = 0x80;
  constexpr unsigned interface_input = 0x81;
  constexpr unsigned endpoint_input = 0x82;
  if ((data[0] & type) != 0 || data[1] != 0) { return std::nullopt; }
  if (data[2] != 0 || data[3] != 0 || data[5] != 0 || data[6] != 2 || data[7] != 0) {
    throw Fault("Unqualified USB GET_STATUS value/index/length");
  }
  if (data[0] == endpoint_input) {
    throw Fault("Unimplemented USB automatic standard request 0x00 recipient=endpoint");
  }
  if ((configuration_status_ & usb::configuration_request) != 0 ||
      (status_ & (usb::setup | usb::setup_overrun)) != 0) {
    throw Fault("Unqualified USB GET_STATUS overlapping firmware SETUP/configuration");
  }
  if (data[0] == interface_input) {
    auto result = interface_selection(data[4], "GET_STATUS");
    result.value = {};
    result.size = 2;
    return result;
  }
  if (data[0] != device_input || data[4] != 0) {
    throw Fault("Unqualified USB GET_STATUS device direction/recipient/index");
  }
  static_cast<void>(current_configuration("GET_STATUS"));
  const auto value =
      ((module_control_ & usb::self_powered) != 0 ? 1U : 0U) | (remote_wakeup_enabled_ ? 2U : 0U);
  return AutomaticRead{.value = {static_cast<std::uint8_t>(value), 0}, .size = 2};
}

std::optional<McuUsb::AutomaticFeature>
McuUsb::decode_feature_setup(const std::array<std::uint8_t, 8> &data) const {
  constexpr unsigned type = 0x60;
  constexpr unsigned clear_feature = 1;
  constexpr unsigned set_feature = 3;
  constexpr unsigned remote_wakeup_feature = 1;
  if ((data[0] & type) != 0 || (data[1] != clear_feature && data[1] != set_feature)) {
    return std::nullopt;
  }
  if (data[0] != 0 || data[2] != remote_wakeup_feature) {
    throw Fault("Unimplemented USB automatic feature recipient/selector");
  }
  if (data[3] != 0 || data[4] != 0 || data[5] != 0 || data[6] != 0 || data[7] != 0) {
    throw Fault("Unqualified USB remote-wakeup feature value/index/length");
  }
  if ((configuration_status_ & usb::configuration_request) != 0 ||
      (status_ & (usb::setup | usb::setup_overrun)) != 0) {
    throw Fault("Unqualified USB feature overlapping firmware SETUP/configuration");
  }
  static_cast<void>(current_configuration("remote-wakeup feature"));
  return AutomaticFeature{.enabled = data[1] == set_feature,
                          .error = (module_control_ & usb::remote_wakeup_capable) == 0};
}
}
