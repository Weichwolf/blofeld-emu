// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "mcu/mcu_usb_registers.hpp"

#include <cstdint>

namespace blofeld {

bool McuUsb::enabled() const {
  return (module_control_ & usb::enable) != 0;
}

void McuUsb::require_control_phase() const {
  if (!control_phase_known_) {
    throw Fault("Unqualified USB control response after endpoint reset; new SETUP required");
  }
}

void McuUsb::set_control(const std::uint16_t value) {
  if (remote_wakeup_enabled_ && ((module_control_ ^ value) & usb::remote_wakeup_capable) != 0) {
    throw Fault("Unqualified USB capability change with enabled remote wakeup");
  }
  if (module_control_ != value) { host_evidence_.link = next_host_revision(host_evidence_.link); }
  module_control_ = value;
}

void McuUsb::require_observation() const {
  if (enabled() && !decoder_known_) { throw Fault("Unknown USB protocol-decoder input state"); }
}

void McuUsb::require_link() const {
  require_observation();
  if (!enabled() || (module_control_ & usb::clock_enable) == 0 ||
      (module_control_ & (usb::phy_suspend | usb::disconnect)) != 0) {
    throw Fault("USB decoded transaction while module/link clocks are inactive");
  }
  if (decoder_.reset || !decoder_.enumerated) {
    throw Fault("USB decoded transaction before reset/enumeration completion or during suspend");
  }
  if (!decoder_.full_speed) { throw Fault("Unimplemented high-speed USB packet path"); }
}

void McuUsb::require_packet() const {
  require_link();
  if (decoder_.suspended) {
    throw Fault("USB decoded transaction before reset/enumeration completion or during suspend");
  }
}

void McuUsb::observe_decoder(const UsbDecoderState state) {
  if ((state.reset && state.suspended) || (state.reset && state.enumerated)) {
    throw Fault("Contradictory USB protocol-decoder observation");
  }
  if (enabled()) {
    if ((module_control_ & usb::clock_enable) == 0 ||
        (module_control_ & (usb::phy_suspend | usb::disconnect)) != 0) {
      throw Fault("Unimplemented USB status observation while module/link clocks are inactive");
    }
    require_observation();
  }
  // USB2.0 9.1.1.6 retains device state through suspend. A decoded suspend/
  // resume transition changes status/IRQs, not the identity of this link session.
  if (!decoder_known_ || decoder_.reset != state.reset || decoder_.enumerated != state.enumerated ||
      decoder_.full_speed != state.full_speed) {
    host_evidence_.link = next_host_revision(host_evidence_.link);
  }
  if (enabled()) {
    if (decoder_.reset != state.reset) { status_ |= usb::reset_change; }
    if (decoder_.suspended != state.suspended) { status_ |= usb::suspend_change; }
    if (state.reset && !decoder_.reset) {
      // USB functional reset: queued packets and active configuration end;
      // static firmware-programmed endpoint descriptions are retained.
      for (auto &buffer : buffers_data_) {
        buffer.control = 0;
        buffer.data1 = false;
        buffer.pid_known = true;
        buffer.transmitted.reset();
      }
      pending_in_.reset();
      ique_controls_.fill(0);
      configuration_status_ = 0;
      alternate_.fill(0);
      setup_stall_ = false;
      control_phase_known_ = true;
      configuration_control_ = false;
      automatic_read_.reset();
      pending_address_.reset();
      device_address_ = 0;
      remote_wakeup_enabled_ = false;
      feature_control_ = false;
      status_ = static_cast<std::uint16_t>(status_ & ~(usb::sof | usb::setup_overrun | usb::setup));
    }
  }
  constexpr unsigned live =
      usb::reset_detected | usb::suspended | usb::enumerated | usb::full_speed;
  status_ = static_cast<std::uint16_t>((status_ & ~live) | (state.reset ? usb::reset_detected : 0) |
                                       (state.suspended ? usb::suspended : 0) |
                                       (state.enumerated ? usb::enumerated : 0) |
                                       (state.full_speed ? usb::full_speed : 0));
  decoder_ = state;
  decoder_known_ = true;
}

void McuUsb::observe_sof(const std::uint16_t frame, const bool crc_valid) {
  require_packet();
  if (frame > usb::frame_mask) { throw Fault("USB frame number exceeds eleven bits"); }
  if (!crc_valid) { return; }
  pending_in_.reset(); // A new valid token ends the preceding IN handshake window.
  frame_ = frame;
  status_ |= usb::sof;
}

void McuUsb::observe_configuration(const UsbConfigurationRequest request) {
  require_packet();
  constexpr unsigned counts_index = 5;
  constexpr unsigned alternates_index = 6;
  if (request.configuration > 2 || request.configuration > (configuration_[counts_index] & 3U)) {
    throw Fault("USB decoder selected an unsupported configuration");
  }
  if ((configuration_status_ & usb::configuration_request) != 0) {
    throw Fault("Unverified overlapping USB endpoint-configuration requests");
  }
  unsigned value = usb::configuration_valid | usb::configuration_request |
                   (static_cast<unsigned>(request.configuration) << 8U);
  if (request.interface) {
    const auto interface = *request.interface;
    if (request.configuration == 0 || interface >= alternate_.size()) {
      throw Fault("Unsupported USB interface request");
    }
    const auto count_shift = request.configuration == 1 ? 4U : 8U;
    const auto count = (configuration_[counts_index] >> count_shift) & 7U;
    if (interface > count || (interface == count && count != 0)) {
      throw Fault("Unsupported or unverified USB interface-count boundary");
    }
    const auto alternate_shift = ((request.configuration - 1U) * 16) + (interface * 4U);
    const auto count_alternates =
        (configuration_[alternates_index] >> alternate_shift) & usb::nibble_mask;
    if (request.alternate >= count_alternates) {
      throw Fault("Unsupported USB alternate-setting request");
    }
    value |= usb::interface_valid | (static_cast<unsigned>(interface) << 4U) | request.alternate;
  } else if (request.alternate != 0) {
    throw Fault("USB alternate setting without an interface request");
  }
  configuration_status_ = static_cast<std::uint16_t>(value);
}

std::uint8_t McuUsb::interrupt_sources() const {
  unsigned result = 0;
  bool endpoint_enable = false;
  for (unsigned index = 0; index < buffers_data_.size(); ++index) {
    const auto control = buffers_data_[index].control;
    endpoint_enable |= (control & usb::transfer_enable) != 0;
    if ((control & usb::transfer_complete) != 0 && (control & usb::transfer_enable) != 0) {
      result |= 1U << (index + 4);
    }
  }
  if (interrupt_mask_ != 0 || endpoint_enable) { require_observation(); }
  if ((status_ & interrupt_mask_ & (usb::suspend_change | usb::reset_change)) != 0 ||
      ((interrupt_mask_ & usb::resume_enable) != 0 && (status_ & usb::suspended) == 0)) {
    result |= 1;
  }
  if ((status_ & interrupt_mask_ & (usb::setup | usb::setup_overrun)) != 0) { result |= 2; }
  if ((configuration_status_ & usb::configuration_request) != 0 &&
      (interrupt_mask_ & usb::configuration_enable) != 0) {
    result |= 4;
  }
  if ((status_ & interrupt_mask_ & usb::sof) != 0) { result |= 8; }
  return static_cast<std::uint8_t>(result);
}

}
