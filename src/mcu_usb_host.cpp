// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"

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
            .type = 0};
  }
  std::optional<UsbEndpointInfo> result;
  for (unsigned index = 2; index < 4; ++index) {
    const auto description = configuration_.at(index - 1);
    if ((description & number_mask) == number && input_endpoint(index) == input) {
      if (result) { throw Fault("Ambiguous USB host endpoint mapping"); }
      result = UsbEndpointInfo{.buffer = static_cast<UsbBuffer>(index),
                               .address = address,
                               .maximum_packet = maximum_packet(index),
                               .type = (description >> 5U) & 3U};
    }
  }
  if (result) { return *result; }
  throw Fault("Unconfigured or unsupported USB host endpoint address=" + hex(address, 2));
}
}
