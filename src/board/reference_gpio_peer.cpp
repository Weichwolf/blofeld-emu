// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_gpio_peer.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <cstdint>
#include <limits>

namespace blofeld::reference {
namespace {
bool owned(const McuPort port, const unsigned mask) {
  constexpr unsigned signal = 1;
  return port == McuPort::q || ((port == McuPort::e || port == McuPort::j) && (mask & signal) != 0);
}
}

void validate_gpio_peer(const Config &config) {
  if (!config.gpio_peer) { return; }
  for (const auto &route : {config.hreq, config.miso}) {
    if (route && owned(route->destination, route->mask)) {
      throw Fault("MCU/DSP GPIO peer destination overlaps an SHI return driver");
    }
  }
  if (config.loopback && owned(config.loopback->destination, config.loopback->destination_mask)) {
    throw Fault("MCU/DSP GPIO peer destination overlaps a loopback driver");
  }
  const auto check = [](const Input &input) {
    const unsigned mask =
        static_cast<unsigned>(input.drive.mask) | input.drive.floating | input.drive.unknown;
    if (input.kind == InputKind::drive && owned(input.port, mask)) {
      throw Fault("MCU/DSP GPIO peer owns Q, J0 and PE0; duplicate external input is unsupported");
    }
  };
  for (const auto &input : config.initial) { check(input); }
  for (const auto &input : config.events) { check(input); }
}

bool Board::gpio_peer_write(const unsigned address, const unsigned size) const {
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
