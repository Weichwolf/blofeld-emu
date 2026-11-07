// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/instrument.hpp"
#include "runtime/mcu_dsp_gpio_peer.hpp"

#include <cstdint>
#include <limits>

namespace blofeld::runtime {
bool Instrument::gpio_peer_write(const unsigned address, const unsigned size) const {
  if (!config_.gpio_peer) { return false; }
  constexpr unsigned last = std::numeric_limits<std::uint16_t>::max();
  for (unsigned offset = 0; offset < size && address <= last && offset <= last - address;
       ++offset) {
    if (McuDspGpioPeer::shared_mcu_write(static_cast<std::uint16_t>(address + offset))) {
      return true;
    }
  }
  return false;
}
}
