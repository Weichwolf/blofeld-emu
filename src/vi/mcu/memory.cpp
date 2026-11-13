// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/memory.hpp"

#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <utility>
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

std::vector<unsigned> boundaries(const std::array<std::uint8_t, 4> &mapping) {
  const unsigned registers = (mapping[1] & register_mask) << 8U;
  const unsigned system =
      ((mapping[0] & system_mask) << 8U) + ((mapping[0] & 1U) != 0 ? high_align : 0);
  const unsigned queue = (mapping[2] & queue_mask) << 8U;
  constexpr unsigned queue_storage = 1536;
  return {0,
          cpu12::address_bytes,
          registers,
          registers + register_bytes,
          system,
          system + system_offset,
          system + system_window,
          queue,
          queue + queue_storage,
          queue + queue_window,
          flash_alias,
          flash_first};
}

struct Area {
  unsigned first, bytes;
  Location location;
  std::uint64_t identity;
};

template <typename Locate, typename Executable>
std::vector<Area> areas(std::vector<unsigned> points, Locate locate, Executable executable) {
  std::ranges::sort(points);
  const auto duplicate = std::ranges::unique(points);
  points.erase(duplicate.begin(), duplicate.end());
  std::vector<Area> result;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const auto first = points[index - 1];
    const auto location = locate(static_cast<std::uint16_t>(first));
    const auto identity = executable(location);
    if (!result.empty()) {
      auto &previous = result.back();
      if (previous.location.space == location.space && previous.identity == identity &&
          unsigned{previous.location.offset} + previous.bytes == location.offset) {
        previous.bytes += points[index] - first;
        continue;
      }
    }
    result.push_back({.first = first,
                      .bytes = points[index] - first,
                      .location = location,
                      .identity = identity});
  }
  return result;
}

void flash_regions(std::vector<cpu12::Ram> &views, FlashImage &flash, const Area &area) {
  const auto known = std::span(flash.known).subspan(area.location.offset, area.bytes);
  std::size_t offset = 0;
  while (offset < known.size()) {
    if (known[offset] == 0) {
      ++offset;
      continue;
    }
    const auto *unknown = static_cast<const std::uint8_t *>(
        std::memchr(known.data() + offset, 0, known.size() - offset));
    const auto end =
        unknown == nullptr ? known.size() : static_cast<std::size_t>(unknown - known.data());
    views.push_back(
        {.first = static_cast<std::uint16_t>(area.first + offset),
         .bytes = std::span(flash.bytes).subspan(area.location.offset + offset, end - offset),
         .writable = false});
    offset = end;
  }
}
}

Memory::Memory(FlashImage &flash) : flash_(flash) {
  reset();
}

void Memory::reset() {
  program_changed_ = program_changed_ || !program_.empty();
  program_.clear();
  program_points_.clear();
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
    if (program_identity(location) != 0 && system_[location.offset] != access.value) {
      program_changed_ = true;
    }
    system_[location.offset] = access.value;
    system_known_[location.offset] = 1;
    return true;
  }
  if (location.space == Space::queue_ram) {
    if (program_identity(location) != 0 && queue_[location.offset] != access.value) {
      program_changed_ = true;
    }
    queue_[location.offset] = access.value;
    queue_known_[location.offset] = 1;
    return true;
  }
  return false;
}

std::uint64_t Memory::program_identity(const Location location) const noexcept {
  for (const auto &range : program_) {
    if (range.space == location.space && location.offset >= range.first &&
        location.offset < range.last) {
      return range.identity;
    }
  }
  return 0;
}

bool Memory::protect(const CodeBinding &binding,
                     const std::span<std::uint64_t> owners,
                     std::vector<unsigned> &points) const {
  if (binding.identity == 0 || binding.code.empty()) {
    throw std::invalid_argument("MCU code binding requires identity and source");
  }
  for (const auto span : binding.code) {
    if (span.words.empty() || span.words.size() > cpu12::address_bytes) {
      throw std::invalid_argument("Invalid MCU code binding extent");
    }
    for (std::size_t index = 0; index < span.words.size(); ++index) {
      const auto address = static_cast<std::uint16_t>(span.address + index);
      std::uint8_t actual = 0;
      if (!read(address, actual) || actual != span.words[index]) { return false; }
      const auto location = locate(address);
      if (location.space != Space::system_ram && location.space != Space::queue_ram) { continue; }
      const auto offset = location.space == Space::system_ram ? std::size_t{location.offset}
                                                              : system_bytes + location.offset;
      auto &identity = owners[offset];
      if (identity != 0 && identity != binding.identity) {
        throw std::invalid_argument("MCU code bindings overlap physical RAM");
      }
      identity = binding.identity;
      points.push_back(address);
      points.push_back(unsigned{address} + 1U);
    }
  }
  return true;
}

bool Memory::bind_program(const std::span<const CodeBinding> bindings) {
  if (program_changed_) { return false; }
  std::array<std::uint64_t, system_bytes + queue_bytes> owners{};
  std::vector<unsigned> points;
  for (const auto &binding : bindings) {
    if (!protect(binding, owners, points)) { return false; }
  }
  std::vector<CodeRange> ranges;
  for (std::size_t index = 0; index < owners.size();) {
    const bool system = index < system_bytes;
    const auto boundary = system ? system_bytes : owners.size();
    if (owners[index] == 0) {
      ++index;
      continue;
    }
    const auto first = index;
    const auto identity = owners[index];
    while (index < boundary && owners[index] == identity) { ++index; }
    const auto offset = system ? 0U : system_bytes;
    ranges.push_back({.space = system ? Space::system_ram : Space::queue_ram,
                      .first = static_cast<std::uint16_t>(first - offset),
                      .last = static_cast<std::uint16_t>(index - offset),
                      .identity = identity});
  }
  program_ = std::move(ranges);
  program_points_ = std::move(points);
  return true;
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
  auto points = boundaries(mapping_);
  points.insert(points.end(), program_points_.begin(), program_points_.end());
  const auto mapped = areas(
      std::move(points),
      [this](const auto address) { return locate(address); },
      [this](const auto location) { return program_identity(location); });
  for (const auto &area : mapped) {
    if (area.location.space == Space::flash) {
      flash_regions(views, flash_, area);
      continue;
    }
    if (area.location.space != Space::system_ram && area.location.space != Space::queue_ram) {
      continue;
    }
    auto view = region(area.location, area.bytes);
    if (area.identity != 0) {
      view.known = {};
      view.writable = false;
      view.identity = area.identity;
    }
    view.first = static_cast<std::uint16_t>(area.first);
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
  const bool changed = mapping_ != pending_ || program_changed_;
  if (changed) {
    program_.clear();
    program_points_.clear();
  }
  program_changed_ = false;
  mapping_ = pending_;
  return changed;
}
}
