// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/io.hpp"

#include <cpu12/core.hpp>
#include <cstdint>

namespace blofeld::vi::mcu {
extern "C" bool
blofeld_mcu_read(void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
  if (owner == nullptr) { return false; }
  const auto callbacks = static_cast<Io *>(owner)->callbacks();
  return callbacks.read(callbacks.owner, address, value);
}

extern "C" bool blofeld_mcu_write(void *owner, const cpu12::ByteWrite access) noexcept {
  if (owner == nullptr) { return false; }
  const auto callbacks = static_cast<Io *>(owner)->callbacks();
  return callbacks.write(callbacks.owner, access);
}

extern "C" bool
blofeld_mcu_read_word(void *owner, const std::uint16_t address, std::uint16_t &value) noexcept {
  if (owner == nullptr) { return false; }
  const auto callbacks = static_cast<Io *>(owner)->callbacks();
  return callbacks.read_word(callbacks.owner, address, value);
}

extern "C" bool blofeld_mcu_write_word(void *owner, const cpu12::WordWrite access) noexcept {
  if (owner == nullptr) { return false; }
  const auto callbacks = static_cast<Io *>(owner)->callbacks();
  return callbacks.write_word(callbacks.owner, access);
}

extern "C" bool blofeld_mcu_read_masked(void *owner,
                                        const cpu12::MaskedByteRead access,
                                        std::uint8_t &value) noexcept {
  if (owner == nullptr) { return false; }
  const auto callbacks = static_cast<Io *>(owner)->callbacks();
  return callbacks.read_masked(callbacks.owner, access, value);
}
}
