// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_usb.hpp"
#include "mcu_usb_registers.hpp"

#include <cstdint>

namespace blofeld {
void McuUsb::reset_endpoints() noexcept {
  for (auto &buffer : buffers_data_) {
    buffer.control = 0;
    buffer.transmitted.reset();
    // The module guide defines the register reset, not the post-EPRST DATA PID.
    buffer.pid_known = false;
  }
  ique_controls_.fill(0);
  pending_in_.reset();
  control_phase_known_ = false;
  constexpr unsigned reset_flags =
      usb::suspend_change | usb::reset_change | usb::setup_overrun | usb::setup;
  status_ = static_cast<std::uint16_t>(status_ & ~reset_flags);
}
}
