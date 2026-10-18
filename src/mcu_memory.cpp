// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_memory.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned register_bytes = 1024;
constexpr unsigned register_base_mask = 0x78;
constexpr unsigned qram_base_mask = 0xf8;
constexpr unsigned smram_base_mask = 0xe0;
constexpr unsigned smram_window = 0x1800;
constexpr unsigned smram_system_offset = 0x0a00;
constexpr unsigned smram_high_alignment = 0x0800;
constexpr unsigned qram_window = 0x0800;
constexpr unsigned flash_base = 0x8000;
constexpr unsigned flash_alias = 0x4000;
constexpr unsigned rom_enable = 1;
constexpr unsigned rom_high_only = 2;
constexpr unsigned map_writable_mask = 0xf9;
constexpr unsigned misc_writable_mask = 0x0f;
constexpr unsigned reset_initram = 0x09;
constexpr unsigned reset_misc = 0x0d;

std::size_t map_index(const std::uint16_t offset) {
  if (offset < hc12::initrm || offset > hc12::misc) {
    throw Fault("Unknown MCU mapping register at " + hex(offset));
  }
  return offset - hc12::initrm;
}
}

McuMemory::McuMemory(const std::span<const std::uint8_t> flash) {
  if (flash.size() != flash_.size()) { throw Fault("The MCU requires a 32768-byte flash image"); }
  std::ranges::copy(flash, flash_.begin());
  reset();
}

void McuMemory::reset() {
  registers_ = {reset_initram, 0, 1, reset_misc};
  pending_ = registers_;
  pending_writes_.reset();
  written_.reset();
}

McuLocation McuMemory::locate(const std::uint16_t address) const {
  const auto registers_base = (registers_[1] & register_base_mask) << 8U;
  if (address >= registers_base && address - registers_base < register_bytes) {
    return {.space = McuSpace::registers,
            .offset = static_cast<std::uint16_t>(address - registers_base)};
  }
  const auto smram_base = ((registers_[0] & smram_base_mask) << 8U) +
                          ((registers_[0] & 1U) != 0 ? smram_high_alignment : 0);
  if (address >= smram_base && address - smram_base < smram_window) {
    const auto relative = address - smram_base;
    if (relative < smram_system_offset) { return {.space = McuSpace::reserved}; }
    return {.space = McuSpace::smram,
            .offset = static_cast<std::uint16_t>(relative - smram_system_offset)};
  }
  const auto qram_base = (registers_[2] & qram_base_mask) << 8U;
  if ((registers_[2] & 1U) != 0 && address >= qram_base && address - qram_base < qram_window) {
    const auto relative = address - qram_base;
    if (relative >= qram_size) { return {.space = McuSpace::reserved}; }
    return {.space = McuSpace::qram, .offset = static_cast<std::uint16_t>(relative)};
  }
  if ((registers_[3] & rom_enable) == 0) { return {}; }
  if (address >= flash_base) {
    return {.space = McuSpace::flash, .offset = static_cast<std::uint16_t>(address - flash_base)};
  }
  if ((registers_[3] & rom_high_only) == 0 && address >= flash_alias) {
    return {.space = McuSpace::flash, .offset = static_cast<std::uint16_t>(address - flash_alias)};
  }
  return {};
}

bool McuMemory::initialized(const McuLocation location) const {
  switch (location.space) {
    case McuSpace::smram: return smram_initialized_.test(location.offset);
    case McuSpace::qram: return qram_initialized_.test(location.offset);
    case McuSpace::flash: return true;
    case McuSpace::reserved:
    case McuSpace::registers:
    case McuSpace::unmapped: return false;
    default: throw Fault("Invalid MCU memory space");
  }
}

std::uint8_t McuMemory::read(const std::uint16_t address) const {
  const auto location = locate(address);
  if (!initialized(location)) {
    throw Fault("Uninitialized or unmapped MCU read at " + hex(address));
  }
  return peek(address);
}

std::uint8_t McuMemory::peek(const std::uint16_t address) const {
  const auto location = locate(address);
  switch (location.space) {
    case McuSpace::smram: return smram_[location.offset];
    case McuSpace::qram: return qram_[location.offset];
    case McuSpace::flash: return flash_[location.offset];
    case McuSpace::reserved:
    case McuSpace::registers:
    case McuSpace::unmapped: return 0;
    default: throw Fault("Invalid MCU memory space");
  }
}

void McuMemory::write(const ByteWrite write) {
  const auto address = write.address;
  const auto value = write.value;
  const auto location = locate(address);
  switch (location.space) {
    case McuSpace::smram:
      smram_[location.offset] = value;
      smram_initialized_.set(location.offset);
      return;
    case McuSpace::qram:
      qram_[location.offset] = value;
      qram_initialized_.set(location.offset);
      return;
    case McuSpace::reserved: throw Fault("Reserved MCU memory write at " + hex(address));
    case McuSpace::flash:
      throw Fault("MCU flash write requires the flash-controller model at " + hex(address));
    case McuSpace::registers:
    case McuSpace::unmapped: throw Fault("Unmapped MCU write at " + hex(address));
    default: throw Fault("Invalid MCU memory space");
  }
}

void McuMemory::check_execute(const std::uint16_t address) const {
  const auto location = locate(address);
  if ((location.space != McuSpace::smram && location.space != McuSpace::qram &&
       location.space != McuSpace::flash) ||
      !initialized(location)) {
    throw Fault("Uninitialized or unmapped MCU instruction fetch at " + hex(address));
  }
}

std::string_view McuMemory::register_name(const std::uint16_t offset) {
  switch (offset) {
    case hc12::initrm: return "INITRM";
    case hc12::initrg: return "INITRG";
    case hc12::initee: return "INITEE";
    case hc12::misc: return "MISC";
    default: return {};
  }
}

std::uint8_t McuMemory::read_register(const std::uint16_t offset) const {
  return registers_.at(map_index(offset));
}

void McuMemory::write_register(const ByteWrite write) {
  const auto offset = write.address;
  const auto value = write.value;
  const auto index = map_index(offset);
  // INITRM/INITRG and MISC are write-once in normal single-chip mode.
  // INITEE follows the MMC default write-anytime policy; chip-specific rights
  // still need calibration against the actual Blofeld MCU integration.
  if (offset != hc12::initee && written_.test(index)) { return; }
  unsigned mask = map_writable_mask;
  if (offset == hc12::initrg) { mask = register_base_mask; }
  if (offset == hc12::misc) { mask = misc_writable_mask; }
  pending_.at(index) = static_cast<std::uint8_t>(value & mask);
  pending_writes_.set(index);
  written_.set(index);
}

bool McuMemory::retire_instruction() {
  const bool changed = pending_writes_.any();
  for (std::size_t index = 0; index < registers_.size(); ++index) {
    if (pending_writes_.test(index)) { registers_[index] = pending_[index]; }
  }
  pending_writes_.reset();
  return changed;
}

}
