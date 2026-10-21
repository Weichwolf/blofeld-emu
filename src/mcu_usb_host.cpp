// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "mcu_usb_registers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
UsbEndpointInfo McuUsb::host_endpoint(const std::uint8_t address) const {
  constexpr unsigned number_mask = 15;
  constexpr unsigned input_bit = 0x80;
  constexpr unsigned reserved_address = 0x70;
  if ((address & reserved_address) != 0) { throw Fault("Reserved USB endpoint address bits"); }
  require_packet();
  const auto number = address & number_mask;
  const auto input = (address & input_bit) != 0;
  if (number == 0) {
    const auto buffer = input ? UsbBuffer::control_in : UsbBuffer::control_out;
    return {.buffer = buffer,
            .address = address,
            .maximum_packet = maximum_packet(input ? 0 : 1),
            .type = 0,
            .data1 =
                buffers_data_.at(static_cast<unsigned>(buffer)).pid_known
                    ? std::optional<bool>(buffers_data_.at(static_cast<unsigned>(buffer)).data1)
                    : std::nullopt};
  }
  std::optional<UsbEndpointInfo> result;
  for (unsigned index = 2; index < 4; ++index) {
    const auto description = configuration_.at(index - 1);
    if ((description & number_mask) == number && input_endpoint(index) == input) {
      if (result) { throw Fault("Ambiguous USB host endpoint mapping"); }
      result = UsbEndpointInfo{.buffer = static_cast<UsbBuffer>(index),
                               .address = address,
                               .maximum_packet = maximum_packet(index),
                               .type = (description >> 5U) & 3U,
                               .data1 = buffers_data_.at(index).pid_known
                                            ? std::optional<bool>(buffers_data_.at(index).data1)
                                            : std::nullopt};
    }
  }
  if (result) { return *result; }
  throw Fault("Unconfigured or unsupported USB host endpoint address=" + hex(address, 2));
}

UsbConfigurationInfo McuUsb::host_configuration() const {
  require_packet();
  if ((configuration_status_ & usb::configuration_request) != 0) {
    throw Fault("USB configuration is pending firmware completion");
  }
  const auto configuration = (configuration_status_ >> 8U) & usb::nibble_mask;
  unsigned count = 0;
  if (configuration != 0) {
    if (configuration > 2 || (configuration_status_ & usb::configuration_valid) == 0) {
      throw Fault("Unqualified USB current configuration status");
    }
    constexpr unsigned counts_index = 5;
    constexpr unsigned second_configuration_shift = 8;
    const auto shift = configuration == 1 ? 4U : second_configuration_shift;
    count = (configuration_[counts_index] >> shift) & 7U;
    if (count == 0 || count > alternate_.size()) {
      throw Fault("Unsupported or ambiguous USB active interface count");
    }
  }
  return {.configuration = static_cast<std::uint8_t>(configuration),
          .interfaces = count,
          .alternatives = alternate_};
}

}
