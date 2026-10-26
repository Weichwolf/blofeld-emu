// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// Explicit hardware-vector fetch capability. The memory owner resolves its
// reset-latched routing and performs the actual read. Logical IRQ selection and
// acknowledgement stay unchanged;ordinary operand/program/reset reads are separate.
class Hcs08VectorMemory {
public:
  virtual ~Hcs08VectorMemory() = default;
  [[nodiscard]] virtual std::uint8_t read_interrupt_vector_byte(std::uint16_t address) = 0;
};
} // namespace blofeld
