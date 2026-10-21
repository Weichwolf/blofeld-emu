// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "mcu_usb_registers.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {

unsigned McuUsb::buffer_index(const UsbBuffer buffer) {
  const auto index = static_cast<unsigned>(buffer);
  if (index >= 4) { throw Fault("Unknown USB local endpoint buffer"); }
  return index;
}

unsigned McuUsb::window_buffer(const std::uint16_t offset) const {
  const auto index = (offset >= usb::buffer_second ? buffers_ >> 4U : buffers_) & 7U;
  if (index >= buffers_data_.size()) { throw Fault("Reserved USB endpoint-buffer selection"); }
  return index;
}

std::optional<std::uint8_t> McuUsb::read_live(const std::uint16_t offset) const {
  switch (offset & ~1U) {
    case usb::status:
      require_observation();
      if (pending_in_ && offset == usb::status) {
        throw Fault("Unverified USB ACTEP read during an in-flight IN transaction");
      }
      return static_cast<std::uint8_t>((offset % 2 == 0 ? status_ >> 8U : status_) &
                                       usb::byte_mask);
    case usb::interrupts:
      return static_cast<std::uint8_t>((offset % 2 == 0 ? interrupt_mask_ >> 8U : interrupt_mask_) &
                                       usb::byte_mask);
    case usb::timestamp:
      require_observation();
      return static_cast<std::uint8_t>((offset % 2 == 0 ? frame_ >> 8U : frame_) & usb::byte_mask);
    case usb::configuration_status:
      require_observation();
      return static_cast<std::uint8_t>(
          (offset % 2 == 0 ? configuration_status_ >> 8U : configuration_status_) & usb::byte_mask);
    default: break;
  }
  if (offset >= usb::endpoint_begin && offset < usb::endpoint_end) {
    require_observation();
    const auto value = buffers_data_.at((offset - usb::endpoint_begin) / 2).control;
    return static_cast<std::uint8_t>((offset % 2 == 0 ? value >> 8U : value) & usb::byte_mask);
  }
  if (usb::ique_control(offset)) {
    require_observation();
    return read_ique_control(offset);
  }
  if (offset >= usb::setup_begin && offset < usb::setup_end) {
    if ((status_ & usb::setup) == 0) { throw Fault("USB setup-buffer contents are not valid"); }
    return setup_.at(offset - usb::setup_begin);
  }
  if (offset >= usb::buffer_begin && offset < usb::buffer_end) {
    const auto &buffer = buffers_data_.at(window_buffer(offset));
    const auto byte = offset % usb::buffer_bytes;
    if (!buffer.known.test(byte)) {
      throw Fault("Unknown USB endpoint-buffer byte at " + hex(offset));
    }
    return buffer.bytes.at(byte);
  }
  return {};
}

void McuUsb::write_endpoint(const ByteWrite access) {
  const auto index = (access.address - usb::endpoint_begin) / 2;
  auto &buffer = buffers_data_.at(index);
  const unsigned old = buffer.control;
  if (buffer.transmitted) {
    throw Fault("Unverified USB endpoint-control update before acknowledgment of transmitted data");
  }
  if (pending_in_ && pending_in_->buffer == index) {
    throw Fault("Unverified USB endpoint-control update during an in-flight IN transaction");
  }
  if (access.address % 2 == 0) {
    const auto requested = static_cast<unsigned>(access.value) << 8U;
    unsigned next = (old & usb::packet_size) |
                    (requested & (usb::stall | usb::data_valid | usb::transfer_enable)) |
                    (old & usb::transfer_complete);
    if ((requested & usb::transfer_complete) != 0) { next &= ~usb::transfer_complete; }
    buffer.control = static_cast<std::uint16_t>(next);
  } else {
    // RXTXSIZ writes are used only by IN endpoints. Nonzero OUT size writes
    // are unspecified by the module guide rather than fabricated receive data.
    if (!input_endpoint(index)) {
      if ((access.value & usb::packet_size) != 0) {
        throw Fault("Unverified USB OUT receive-size write");
      }
      return;
    }
    buffer.control =
        static_cast<std::uint16_t>((old & ~usb::packet_size) | (access.value & usb::packet_size));
  }
}

void McuUsb::acknowledge_configuration(const ByteWrite access) {
  if (access.address != usb::configuration_status ||
      (access.value & (usb::configuration_done >> 8U)) == 0 ||
      (configuration_status_ & usb::configuration_request) == 0) {
    return;
  }
  const auto interface = (configuration_status_ >> 4U) & usb::nibble_mask;
  if ((configuration_status_ & usb::interface_valid) != 0) {
    alternate_.at(interface) = static_cast<std::uint8_t>(configuration_status_ & usb::nibble_mask);
  } else {
    alternate_.fill(0);
  }
  for (unsigned index = 2; index < buffers_data_.size(); ++index) {
    const auto selected_interface = (configuration_.at(index - 1) >> 11U) & usb::nibble_mask;
    if ((configuration_status_ & usb::interface_valid) == 0 || selected_interface == interface) {
      buffers_data_[index].data1 = false;
      buffers_data_[index].pid_known = true;
      buffers_data_[index].transmitted.reset();
      if (pending_in_ && pending_in_->buffer == index) { pending_in_.reset(); }
    }
  }
  configuration_status_ = static_cast<std::uint16_t>(
      configuration_status_ & ~(usb::configuration_done | usb::configuration_request));
}

bool McuUsb::write_live(const ByteWrite access) {
  const auto offset = access.address;
  switch (offset & ~1U) {
    case usb::status: {
      const unsigned value =
          offset % 2 == 0 ? static_cast<unsigned>(access.value) << 8U : access.value;
      status_ = static_cast<std::uint16_t>(status_ & ~(value & usb::clearable_status));
      return true;
    }
    case usb::interrupts: {
      const unsigned next = offset % 2 == 0 ? (static_cast<unsigned>(access.value) << 8U) |
                                                  (interrupt_mask_ & usb::byte_mask)
                                            : (interrupt_mask_ & ~usb::byte_mask) | access.value;
      interrupt_mask_ = static_cast<std::uint16_t>(next & usb::interrupt_mask);
      return true;
    }
    case usb::timestamp: return true;
    case usb::configuration_status: acknowledge_configuration(access); return true;
    default: break;
  }
  if (offset >= usb::endpoint_begin && offset < usb::endpoint_end) {
    write_endpoint(access);
    return true;
  }
  if (usb::ique_control(offset)) {
    write_ique_control(access);
    return true;
  }
  if (offset >= usb::setup_begin && offset < usb::setup_end) {
    throw Fault("Unimplemented USB setup-buffer CPU write");
  }
  if (offset >= usb::buffer_begin && offset < usb::buffer_end) {
    const auto index = window_buffer(offset);
    if (pending_in_ && pending_in_->buffer == index) {
      throw Fault("Unverified USB buffer modification during an in-flight IN transaction");
    }
    auto &buffer = buffers_data_.at(index);
    if (buffer.transmitted) {
      throw Fault("USB IN buffer remains owned by the unacknowledged transmitted packet");
    }
    const auto byte = offset % usb::buffer_bytes;
    buffer.bytes.at(byte) = access.value;
    buffer.known.set(byte);
    return true;
  }
  return false;
}

}
