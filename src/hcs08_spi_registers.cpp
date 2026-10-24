// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08_spi.hpp"

#include <cstdint>

namespace blofeld {
namespace spi = hcs08_spi;

namespace {
constexpr unsigned format_mask = 0x1d, baud_mask = 0x77, control2_mask = 0x1b;
constexpr unsigned unsupported_control2 = 0x13;
}

std::uint8_t Hcs08Spi::read(const std::uint16_t address) {
  healthy();
  switch (address) {
    case spi::control1: return state_.control1;
    case spi::control2: return state_.control2;
    case spi::baud: return state_.baud;
    case spi::status: {
      const auto value = static_cast<std::uint8_t>((state_.receive_full ? spi::receive_full : 0U) |
                                                   (!state_.transmit ? spi::transmit_empty : 0U));
      state_.receive_armed |= state_.receive_full;
      state_.transmit_armed |= !state_.transmit;
      return value;
    }
    case spi::data: {
      const auto value = state_.receive;
      if (state_.receive_armed) { state_.receive_full = false; }
      state_.receive_armed = false;
      return value;
    }
    default: fail("read", address, "reserved or unsupported register");
  }
}

void Hcs08Spi::control(const std::uint8_t value) {
  const bool enabled = (value & spi::enable) != 0;
  if (enabled && (value & spi::master) == 0) {
    fail("write", spi::control1, "slave SPI pin/selection ownership is unqualified");
  }
  if (enabled && (state_.active || state_.transmit) &&
      ((value ^ state_.control1) & format_mask) != 0) {
    fail("write", spi::control1, "active format/role change is unqualified");
  }
  state_.control1 = value;
  if (!enabled) {
    state_.transmit.reset();
    state_.active = false;
    state_.receive_full = false;
    state_.transmit_armed = false;
    state_.receive_armed = false;
    state_.edges = 0;
    state_.samples = 0;
    state_.mosi.reset();
  }
}

void Hcs08Spi::write(const ByteWrite access) {
  healthy();
  switch (access.address) {
    case spi::control1: control(access.value); return;
    case spi::control2:
      if ((access.value & unsupported_control2) != 0) {
        fail(
            "write", access.address, "mode fault, bidirectional or wait-clock mode is unqualified");
      }
      state_.control2 = static_cast<std::uint8_t>(access.value & control2_mask);
      return;
    case spi::baud: state_.baud = static_cast<std::uint8_t>(access.value & baud_mask); return;
    case spi::status: return; // Read-only flags; no W1C or poll-driven progress.
    case spi::data:
      if (!state_.transmit_armed || state_.transmit) { return; }
      if ((state_.control1 & spi::enable) == 0) {
        fail("write", access.address, "disabled transmit-buffer write is unqualified");
      }
      state_.transmit = access.value;
      state_.transmit_armed = false;
      return;
    default: fail("write", access.address, "reserved or unsupported register");
  }
}
} // namespace blofeld
