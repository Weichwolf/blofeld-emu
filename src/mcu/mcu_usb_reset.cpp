// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_usb.hpp"
#include "mcu/mcu_usb_registers.hpp"

#include <cstdint>

namespace blofeld {
void McuUsb::reset_endpoints() {
  host_evidence_.link = next_host_revision(host_evidence_.link);
  for (auto &buffer : buffers_data_) {
    buffer.control = 0;
    buffer.transmitted.reset();
    // The module guide defines the register reset, not the post-EPRST DATA PID.
    buffer.pid_known = false;
    // EPRST documents CSR reset, not whether an existing functional halt ends.
    if (buffer.functional_halt.value_or(true)) { buffer.functional_halt.reset(); }
  }
  ique_controls_.fill(0);
  pending_in_.reset();
  automatic_read_.reset();
  pending_address_.reset();
  feature_control_ = false;
  control_phase_known_ = false;
  constexpr unsigned reset_flags =
      usb::suspend_change | usb::reset_change | usb::setup_overrun | usb::setup;
  status_ = static_cast<std::uint16_t>(status_ & ~reset_flags);
}
}
