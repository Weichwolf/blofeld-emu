// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "keyboard/hcs08_vector_memory.hpp"

#include <cstdint>

namespace blofeld::detail::hcs08 {
[[nodiscard]] inline std::uint8_t vector_byte(Bus &bus, const std::uint16_t address) {
  if (auto *const owner = dynamic_cast<Hcs08VectorMemory *>(&bus); owner != nullptr) {
    return owner->read_interrupt_vector_byte(address);
  }
  return static_cast<std::uint8_t>(bus.read_memory({.address = address}));
}
}
