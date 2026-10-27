// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "mcu/mcu_usb_registers.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace {
std::optional<unsigned> status_buffer(const std::array<std::uint32_t, 7> &descriptions,
                                      const UsbConfigurationInfo &configuration,
                                      const std::uint8_t address) {
  constexpr unsigned number_mask = 15;
  constexpr unsigned input_bit = 0x80;
  constexpr unsigned direction_bit = 0x10;
  constexpr unsigned field_mask = 15;
  std::optional<unsigned> result;
  for (unsigned index = 1; index < 5; ++index) {
    const auto description = descriptions.at(index);
    const auto input = (description & direction_bit) != 0;
    const auto selected = (description >> 7U) & field_mask;
    if ((description & number_mask) != (address & number_mask) ||
        input != ((address & input_bit) != 0) || configuration.configuration == 0 ||
        selected != configuration.configuration) {
      continue;
    }
    const auto interface = (description >> 11U) & field_mask;
    if (interface >= configuration.interfaces || interface >= configuration.alternatives.size()) {
      throw Fault("Unqualified USB endpoint status interface outside active configuration");
    }
    const auto alternate = (description >> 15U) & field_mask;
    if (alternate != configuration.alternatives.at(interface)) { continue; }
    if (result) { throw Fault("Ambiguous USB endpoint status mapping"); }
    result = index + 1;
  }
  return result;
}
}

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
  if ((configuration_status_ & usb::configuration_request) != 0 ||
      (status_ & (usb::setup | usb::setup_overrun)) != 0) {
    throw Fault("Unqualified USB GET_STATUS overlapping firmware SETUP/configuration");
  }
  if (data[0] == endpoint_input) {
    constexpr unsigned reserved_address = 0x70;
    constexpr unsigned number_mask = 15;
    static_cast<void>(current_configuration("endpoint GET_STATUS"));
    if ((data[4] & reserved_address) != 0) {
      throw Fault("Reserved USB endpoint GET_STATUS address bits");
    }
    if ((data[4] & number_mask) == 0) {
      throw Fault("Unqualified USB default control endpoint functional-halt status");
    }
    const auto index = status_buffer(configuration_, host_configuration(), data[4]);
    if (!index) { return AutomaticRead{.size = 2, .error = true}; }
    if (*index >= buffers_data_.size()) {
      throw Fault("Unimplemented USB endpoint halt status physical-endpoint=" +
                  std::to_string(*index));
    }
    static_cast<void>(maximum_packet(*index));
    const auto &buffer = buffers_data_.at(*index);
    const auto commanded = (buffer.control & usb::stall) != 0;
    if (!buffer.functional_halt && !commanded) {
      throw Fault("Unqualified USB functional-halt status after endpoint reset physical-endpoint=" +
                  std::to_string(*index));
    }
    const auto halted = commanded || buffer.functional_halt.value_or(false);
    return AutomaticRead{.value = {static_cast<std::uint8_t>(halted), 0}, .size = 2};
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
