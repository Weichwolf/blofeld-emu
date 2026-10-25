// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace {
void validate_transfer(const MemoryRead access) {
  constexpr unsigned address_space = 65536;
  if (access.size == 0 || access.size > sizeof(std::uint64_t) ||
      access.size > address_space - access.address) {
    throw Fault("Unsupported MCU operand range at " + hex(access.address) +
                " size=" + std::to_string(access.size));
  }
}
}

std::uint64_t Bus::read_memory(const MemoryRead access) {
  validate_transfer(access);
  std::uint64_t result = 0;
  for (unsigned index = 0; index < access.size; ++index) {
    const auto address = static_cast<std::uint16_t>(access.address + index);
    result = (result << 8U) | read(address);
  }
  return result;
}

void Bus::write_memory(const MemoryWrite access) {
  validate_transfer({.address = access.address, .size = access.size});
  constexpr unsigned byte_mask = 0xff;
  for (unsigned index = 0; index < access.size; ++index) {
    const auto shift = (access.size - index - 1) * 8;
    const auto address = static_cast<std::uint16_t>(access.address + index);
    write(address, static_cast<std::uint8_t>((access.value >> shift) & byte_mask));
  }
}

}
