// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>

namespace blofeld {
struct MaskedByteRead {
  std::uint16_t address = 0;
  std::uint8_t mask = 0;
};

// The ordinary backend still performs one real byte read with all its effects.
[[nodiscard]] inline std::uint8_t read_masked_byte(Bus &bus, const MaskedByteRead access) {
  const auto value = bus.read_memory({.address = access.address});
  constexpr unsigned byte_mask = 0xff;
  if (value > byte_mask) {
    throw Fault("Masked byte read exceeds eight bits at " + hex(access.address));
  }
  return static_cast<std::uint8_t>(value & access.mask);
}

// Optional native selected-bit interface. Unselected result bits are zero and
// convey no knowledge about the rest of the byte. The ordinary Bus ABI is kept.
class MaskedByteBus : public Bus {
public:
  [[nodiscard]] virtual std::uint8_t read_masked_byte(const MaskedByteRead access) {
    return blofeld::read_masked_byte(*this, access);
  }
};

[[nodiscard]] inline std::uint8_t observe_masked_byte(Bus &bus, const MaskedByteRead access) {
  if (auto *selected = dynamic_cast<MaskedByteBus *>(&bus)) {
    return selected->read_masked_byte(access);
  }
  return read_masked_byte(bus, access);
}
}
