// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"

#include <memory>

namespace blofeld {
McuBus::McuBus(const McuBus &source, CloneState /*tag*/)
    : revision_(source.revision_),
      code_revision_(source.code_revision_),
      decoded_bytes_(source.decoded_bytes_),
      memory_(source.memory_),
      flash_(memory_, source.flash_),
      gpio_(source.gpio_),
      clock_(source.clock_),
      sci_(source.sci_),
      timer_(source.timer_),
      usb_(source.usb_),
      interrupts_(source.interrupts_),
      context_(source.context_) {}

std::unique_ptr<McuBus> McuBus::clone() const {
  if (notifying_) { throw Fault("Cannot clone during an active MCU bus observation"); }
  // The flash object must bind to this clone's memory, never the source's array.
  return std::unique_ptr<McuBus>(new McuBus(*this, CloneState{}));
}
}
