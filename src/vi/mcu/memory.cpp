// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/memory.hpp"

#include "storage/flash_image.hpp"

#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace blofeld::vi::mcu {
namespace {
constexpr unsigned register_bytes = 1024;
constexpr unsigned register_mask = 0x78;
constexpr unsigned system_mask = 0xe0;
constexpr unsigned system_window = 0x1800;
constexpr unsigned system_offset = 0x0a00;
constexpr unsigned high_align = 0x0800;
constexpr unsigned queue_mask = 0xf8;
constexpr unsigned queue_window = 0x0800;
constexpr unsigned flash_first = 0x8000;
constexpr unsigned flash_alias = 0x4000;
constexpr unsigned mapping_first = 0x0010;
constexpr std::array<std::uint8_t, 4> write_masks{0xf9, register_mask, 0xf9, 0x0f};

std::size_t mapping_index(const std::uint16_t offset) {
  if (offset < mapping_first || offset >= mapping_first + write_masks.size()) {
    throw std::out_of_range("Unknown UF32 memory-mapping register");
  }
  return offset - mapping_first;
}
}

Memory::Memory(FlashImage &flash) : flash_(flash) {
  reset();
}

void Memory::reset() {
  mapping_ = {9, 0, 1, 0x0d};
  pending_ = mapping_;
  written_ = 0;
}

void Memory::power_on() {
  system_.fill(0);
  system_known_.fill(0);
  queue_.fill(0);
  queue_known_.fill(0);
  reset();
}

Location Memory::locate(const std::uint16_t address) const noexcept {
  const unsigned registers = (mapping_[1] & register_mask) << 8U;
  if (address >= registers && address - registers < register_bytes) {
    return {.space = Space::registers, .offset = static_cast<std::uint16_t>(address - registers)};
  }
  const unsigned system =
      ((mapping_[0] & system_mask) << 8U) + ((mapping_[0] & 1U) != 0 ? high_align : 0);
  if (address >= system && address - system < system_window) {
    const auto relative = address - system;
    return relative < system_offset
               ? Location{.space = Space::reserved}
               : Location{.space = Space::system_ram,
                          .offset = static_cast<std::uint16_t>(relative - system_offset)};
  }
  const unsigned queue = (mapping_[2] & queue_mask) << 8U;
  if ((mapping_[2] & 1U) != 0 && address >= queue && address - queue < queue_window) {
    const auto relative = address - queue;
    return relative >= queue_bytes ? Location{.space = Space::reserved}
                                   : Location{.space = Space::queue_ram,
                                              .offset = static_cast<std::uint16_t>(relative)};
  }
  if ((mapping_[3] & 1U) == 0) { return {}; }
  if (address >= flash_first) {
    return {.space = Space::flash, .offset = static_cast<std::uint16_t>(address - flash_first)};
  }
  if ((mapping_[3] & 2U) == 0 && address >= flash_alias) {
    return {.space = Space::flash, .offset = static_cast<std::uint16_t>(address - flash_alias)};
  }
  return {};
}

bool Memory::read(const std::uint16_t address, std::uint8_t &value) const noexcept {
  const auto location = locate(address);
  const auto at = location.offset;
  switch (location.space) {
    case Space::flash:
      if (flash_.known[at] == 0) { return false; }
      value = flash_.bytes[at];
      return true;
    case Space::system_ram:
      if (system_known_[at] == 0) { return false; }
      value = system_[at];
      return true;
    case Space::queue_ram:
      if (queue_known_[at] == 0) { return false; }
      value = queue_[at];
      return true;
    case Space::unmapped:
    case Space::registers:
    case Space::reserved:
    default: return false;
  }
}

bool Memory::write(const cpu12::ByteWrite access) noexcept {
  const auto location = locate(access.address);
  if (location.space == Space::system_ram) {
    system_[location.offset] = access.value;
    system_known_[location.offset] = 1;
    return true;
  }
  if (location.space == Space::queue_ram) {
    queue_[location.offset] = access.value;
    queue_known_[location.offset] = 1;
    return true;
  }
  return false;
}

cpu12::Ram Memory::region(const Location start, const std::size_t size) {
  if (start.space == Space::flash) {
    return {.bytes = std::span(flash_.bytes).subspan(start.offset, size), .writable = false};
  }
  const auto bytes = start.space == Space::system_ram ? std::span<std::uint8_t>(system_)
                                                      : std::span<std::uint8_t>(queue_);
  const auto known = start.space == Space::system_ram ? std::span<std::uint8_t>(system_known_)
                                                      : std::span<std::uint8_t>(queue_known_);
  return {.bytes = bytes.subspan(start.offset, size), .known = known.subspan(start.offset, size)};
}

std::vector<cpu12::Ram> Memory::regions() {
  std::vector<cpu12::Ram> views;
  constexpr unsigned end = cpu12::address_bytes;
  const auto grantable = [this](const Location at) {
    return at.space == Space::system_ram || at.space == Space::queue_ram ||
           (at.space == Space::flash && flash_.known[at.offset] != 0);
  };
  unsigned address = 0;
  while (address < end) {
    const auto start = locate(static_cast<std::uint16_t>(address));
    if (!grantable(start)) {
      ++address;
      continue;
    }
    const auto first = address++;
    while (address < end) {
      const auto next = locate(static_cast<std::uint16_t>(address));
      if (next.space != start.space || next.offset != start.offset + address - first ||
          !grantable(next)) {
        break;
      }
      ++address;
    }
    auto view = region(start, address - first);
    view.first = static_cast<std::uint16_t>(first);
    views.push_back(view);
  }
  return views;
}

std::uint8_t Memory::read_mapping(const std::uint16_t offset) const {
  return mapping_[mapping_index(offset)];
}

bool Memory::write_mapping(const cpu12::ByteWrite access) {
  const auto index = mapping_index(access.address);
  const auto bit = static_cast<std::uint8_t>(1U << index);
  constexpr unsigned queue_mapping = 2;
  if (index != queue_mapping && (written_ & bit) != 0) { return false; }
  written_ |= bit;
  pending_[index] = static_cast<std::uint8_t>(access.value & write_masks[index]);
  return pending_ != mapping_;
}

bool Memory::commit_mapping() noexcept {
  const bool changed = mapping_ != pending_;
  mapping_ = pending_;
  return changed;
}
}
