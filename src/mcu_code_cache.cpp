// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"

#include <cstdint>

namespace blofeld {

void McuBus::note_code_byte(const std::uint16_t address) const {
  decoded_bytes_.set(address);
}

void McuBus::invalidate_code(const MemoryRead access) {
  for (unsigned index = 0; index < access.size; ++index) {
    if (decoded_bytes_.test(static_cast<unsigned>(access.address) + index)) {
      ++code_revision_;
      return;
    }
  }
}

}
