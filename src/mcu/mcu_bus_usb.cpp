// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"

#include <array>
#include <cstdint>

namespace blofeld {
void McuBus::observe_usb_decoder(const UsbDecoderState state) {
  usb_.observe_decoder(state);
}

void McuBus::observe_usb_sof(const std::uint16_t frame, const bool crc_valid) {
  usb_.observe_sof(frame, crc_valid);
}

void McuBus::observe_usb_configuration(const UsbConfigurationRequest request) {
  usb_.observe_configuration(request);
}

UsbResponse McuBus::receive_usb_setup(const std::array<std::uint8_t, 8> &data,
                                      const bool crc_valid) {
  return usb_.receive_setup(data, crc_valid);
}

UsbResponse McuBus::receive_usb_out(const UsbOutPacket packet) {
  return usb_.receive_out(packet);
}

UsbInPacket McuBus::request_usb_in(const UsbBuffer buffer) {
  return usb_.request_in(buffer);
}

void McuBus::finish_usb_in(const bool acknowledged) {
  usb_.finish_in(acknowledged);
}

UsbEndpointInfo McuBus::usb_host_endpoint(const std::uint8_t address) const {
  return usb_.host_endpoint(address);
}

UsbConfigurationInfo McuBus::usb_host_configuration() const {
  return usb_.host_configuration();
}

}
