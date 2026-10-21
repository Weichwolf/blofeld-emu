// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "mcu_usb_registers.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld {

bool McuUsb::input_endpoint(const unsigned index) const {
  if (index < 2) { return index == 0; }
  return (configuration_.at(index - 1) & usb::input_direction) != 0;
}

unsigned McuUsb::maximum_packet(const unsigned index) const {
  const auto description = configuration_.at(index < 2 ? 0 : index - 1);
  const auto maximum = (description >> usb::maximum_packet_shift) & usb::frame_mask;
  if (maximum == 0 || maximum > usb::buffer_bytes) {
    throw Fault("Unsupported USB local maximum packet size");
  }
  if (index < 2) {
    constexpr unsigned packet_field = usb::frame_mask << usb::maximum_packet_shift;
    if ((description & ~packet_field) != 0 ||
        (maximum != 8 && maximum != 16 && maximum != 32 && maximum != usb::buffer_bytes)) {
      throw Fault("Unsupported default USB endpoint description");
    }
  } else {
    constexpr unsigned isochronous_shift = 30;
    if ((description >> isochronous_shift) != 0) {
      throw Fault("Reserved USB local endpoint NISOF field physical-endpoint=" +
                  std::to_string(index) + " description=" + hex(description, 8));
    }
    const auto type = (description >> 5U) & 3U;
    const auto configuration = (description >> 7U) & usb::nibble_mask;
    const auto interface = (description >> 11U) & usb::nibble_mask;
    const auto alternate = (description >> 15U) & usb::nibble_mask;
    const auto number = description & usb::nibble_mask;
    const auto selected = (configuration_status_ >> 8U) & usb::nibble_mask;
    if (type < 2 || number == 0 || number > 3 || configuration == 0 || configuration != selected ||
        (configuration_status_ & usb::configuration_valid) == 0 ||
        (configuration_status_ & usb::configuration_request) != 0 ||
        interface >= alternate_.size() || alternate_.at(interface) != alternate) {
      throw Fault("Inactive or unsupported USB physical endpoint description");
    }
    if (number == 3) {
      for (unsigned other = 1; other < 5; ++other) {
        if (other != index - 1 && (configuration_[other] & usb::nibble_mask) == 3) {
          throw Fault("USB endpoint number 3 is assigned to more than one physical endpoint");
        }
      }
    }
  }
  return maximum;
}

UsbResponse McuUsb::stall_response(const unsigned index) {
  auto &buffer = buffers_data_.at(index);
  if (index >= 2) { buffer.control = static_cast<std::uint16_t>(buffer.control & ~usb::stall); }
  return UsbResponse::stall;
}

UsbResponse McuUsb::receive_setup(const std::array<std::uint8_t, 8> &data, const bool crc_valid) {
  require_packet();
  pending_in_.reset();
  if (!crc_valid) { return UsbResponse::none; }
  if (configuration_control_ && (configuration_status_ & usb::configuration_request) != 0) {
    throw Fault("Unqualified USB SETUP replacing an unacknowledged configuration request");
  }
  const auto type = (data[0] >> 5U) & 3U;
  constexpr unsigned get_descriptor = 6;
  constexpr unsigned set_descriptor = 7;
  constexpr unsigned synch_frame = 12;
  if (type == 3) { throw Fault("Reserved USB SETUP request type"); }
  const auto configuration = decode_configuration_setup(data);
  const bool descriptor = type == 0 && data[1] == get_descriptor;
  const bool optional = type == 0 && (data[1] == set_descriptor || data[1] == synch_frame);
  if (type == 0 && !descriptor && !optional && !configuration) {
    throw Fault("Unimplemented USB automatic standard request " + hex(data[1], 2));
  }
  static_cast<void>(maximum_packet(0));
  if (configuration && (status_ & (usb::setup | usb::setup_overrun)) != 0) {
    throw Fault("Unqualified USB configuration SETUP while firmware SETUP flags remain pending");
  }
  if (configuration) { observe_configuration(*configuration); }
  for (unsigned index = 0; index < 2; ++index) {
    auto &buffer = buffers_data_[index];
    buffer.control = static_cast<std::uint16_t>(buffer.control & ~(usb::stall | usb::data_valid));
    buffer.data1 = true;
    buffer.pid_known = true;
    buffer.transmitted.reset();
  }
  setup_stall_ =
      optional && (module_control_ & (data[1] == set_descriptor ? usb::descriptor_support
                                                                : usb::synch_frame_support)) == 0;
  control_phase_known_ = true;
  configuration_control_ = configuration.has_value();
  if (setup_stall_ || configuration_control_) { return UsbResponse::ack; }
  if ((status_ & usb::setup) != 0) { status_ |= usb::setup_overrun; }
  setup_ = data;
  status_ |= usb::setup;
  return UsbResponse::ack;
}

UsbResponse McuUsb::receive_out(const UsbOutPacket packet) {
  require_packet();
  const auto index = buffer_index(packet.buffer);
  if (input_endpoint(index)) { throw Fault("USB OUT decoder routed to an IN-only endpoint"); }
  pending_in_.reset();
  if (!packet.crc_valid) { return UsbResponse::none; }
  const auto maximum = maximum_packet(index);
  if (index == 1 && configuration_control_) {
    require_control_phase();
    throw Fault("USB OUT during automatic configuration status-IN phase");
  }
  if (packet.data.size() > maximum) {
    throw Fault("USB OUT packet exceeds the programmed endpoint size");
  }
  if (index == 1 && setup_stall_) {
    require_control_phase();
    return UsbResponse::stall;
  }
  auto &buffer = buffers_data_[index];
  if ((buffer.control & usb::stall) != 0) { return stall_response(index); }
  if (!buffer.pid_known) {
    throw Fault("Unknown USB OUT DATA PID after endpoint reset at physical endpoint " +
                std::to_string(index));
  }
  // A lost ACK produces a duplicate PID. It is acknowledged without delivering
  // the packet again, even while the original OUT buffer remains occupied.
  if (packet.data1 != buffer.data1) { return UsbResponse::ack; }
  if ((buffer.control & usb::data_valid) != 0) { return UsbResponse::nak; }
  for (unsigned byte = 0; byte < packet.data.size(); ++byte) {
    buffer.bytes[byte] = packet.data[byte];
    buffer.known.set(byte);
  }
  buffer.control = static_cast<std::uint16_t>((buffer.control & ~usb::packet_size) |
                                              usb::data_valid | usb::transfer_complete |
                                              static_cast<unsigned>(packet.data.size()));
  buffer.data1 = !buffer.data1;
  return UsbResponse::ack;
}

UsbInPacket McuUsb::request_in(const UsbBuffer endpoint) {
  require_packet();
  const auto index = buffer_index(endpoint);
  if (!input_endpoint(index)) { throw Fault("USB IN decoder routed to an OUT-only endpoint"); }
  const auto maximum = maximum_packet(index);
  if (pending_in_ && pending_in_->buffer != index) { pending_in_.reset(); }
  if (index == 0 && setup_stall_) {
    require_control_phase();
    return {.response = UsbResponse::stall};
  }
  if (index == 0 && configuration_control_) {
    require_control_phase();
    if ((configuration_status_ & usb::configuration_request) != 0) {
      return {.response = UsbResponse::nak};
    }
    throw Fault("Unqualified USB automatic configuration status completion after DONEECRU");
  }
  auto &buffer = buffers_data_[index];
  if ((buffer.control & usb::stall) != 0) { return {.response = stall_response(index)}; }
  if ((buffer.control & usb::data_valid) == 0) { return {.response = UsbResponse::nak}; }
  if (pending_in_ && pending_in_->buffer == index) { return pending_in_->packet; }
  if (buffer.transmitted) {
    pending_in_ = PendingIn{.buffer = index, .packet = *buffer.transmitted};
    return *buffer.transmitted;
  }
  const auto size = buffer.control & usb::packet_size;
  if (size > maximum) { throw Fault("USB IN packet exceeds the programmed endpoint size"); }
  if (!buffer.pid_known) {
    throw Fault("Unknown USB IN DATA PID after endpoint reset at physical endpoint " +
                std::to_string(index));
  }
  UsbInPacket packet{.response = UsbResponse::data, .size = size, .data1 = buffer.data1};
  for (unsigned byte = 0; byte < size; ++byte) {
    if (!buffer.known.test(byte)) { throw Fault("Unknown USB IN endpoint-buffer data"); }
    packet.data[byte] = buffer.bytes[byte];
  }
  pending_in_ = PendingIn{.buffer = index, .packet = packet};
  buffer.transmitted = packet;
  return packet;
}

void McuUsb::finish_in(const bool acknowledged) {
  if (!pending_in_) { throw Fault("USB IN handshake without an outstanding data packet"); }
  require_packet();
  if (acknowledged) {
    auto &buffer = buffers_data_.at(pending_in_->buffer);
    buffer.control =
        static_cast<std::uint16_t>((buffer.control & ~usb::data_valid) | usb::transfer_complete);
    buffer.data1 = !buffer.data1;
    buffer.transmitted.reset();
  }
  pending_in_.reset();
}

}
