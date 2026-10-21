// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "mcu_usb_registers.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace {
unsigned control_index(const std::uint16_t offset) {
  if ((offset & ~1U) == usb::ique_out_control) { return 0; }
  if ((offset & ~1U) == usb::ique_in_control) { return 1; }
  throw Fault("Unknown USB IQUE control address " + hex(offset));
}
}

void McuUsb::require_unconfigured_ique(const unsigned index) const {
  constexpr unsigned first_ique_description = 3;
  if (configuration_.at(first_ique_description + index) != 0) {
    throw Fault("Unimplemented configured USB IQUE endpoint " + std::to_string(index + 4));
  }
}

std::uint8_t McuUsb::read_ique_control(const std::uint16_t offset) const {
  const auto index = control_index(offset);
  require_unconfigured_ique(index);
  const auto value = ique_controls_.at(index);
  return static_cast<std::uint8_t>((offset % 2 == 0 ? value >> 8U : value) & usb::byte_mask);
}

void McuUsb::write_ique_control(const ByteWrite access) {
  const auto index = control_index(access.address);
  require_unconfigured_ique(index);
  auto &control = ique_controls_.at(index);
  const unsigned old = control;
  if (access.address % 2 == 0) {
    const auto requested = static_cast<unsigned>(access.value) << 8U;
    unsigned next =
        (old & usb::byte_mask) | (requested & usb::ique_writable) | (old & usb::transfer_complete);
    if ((requested & usb::transfer_complete) != 0) { next &= ~usb::transfer_complete; }
    control = static_cast<std::uint16_t>(next);
  } else {
    control = static_cast<std::uint16_t>((old & ~usb::byte_mask) | access.value);
  }
}
}
