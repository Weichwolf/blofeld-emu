// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>

namespace blofeld {
struct Hcs08KnownByte {
  static constexpr std::uint8_t all = 0xff;
  std::uint8_t value = 0;
  std::uint8_t known = all;
};

// Optional board capability for genuine unknown register bits stored in RAM.
// Ordinary Bus reads/fetches must still reject bytes with unknown bits. These
// operations retain real access checks/effects; they are not peek/zero-fill APIs.
// Qualified untimed LDA/STA, raw-copy RTI and unused-value CLR consumers retain
// masks; address permissions and peripheral effects remain bus-owned. Captured
// program-period LDA/STA still requires fully known data until separately owned.
class Hcs08KnownMemory {
public:
  virtual ~Hcs08KnownMemory() = default;
  [[nodiscard]] virtual Hcs08KnownByte read_known_byte(std::uint16_t address) = 0;
  virtual void write_known_byte(ByteWrite access, std::uint8_t known) = 0;
};
} // namespace blofeld
