// SPDX-License-Identifier: GPL-3.0-only
#include "storage/media_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld::vi {
namespace {
constexpr std::array<std::uint8_t, 8> magic{'B', 'L', 'F', 'N', 'A', 'N', 'D', '\n'};
constexpr std::size_t header_bytes = 32, page_count_offset = 16;
constexpr unsigned block_valid = 1, program_protected = 2, erase_protected = 4;
constexpr std::size_t block_count_offset = 20, page_size_offset = 24, reserved_offset = 28;
constexpr std::uint8_t all_bits = 0xff;
constexpr auto missing = std::numeric_limits<std::uint32_t>::max();

void store(const std::span<std::uint8_t> bytes,
           const std::size_t at,
           const std::uint32_t value) noexcept {
  for (unsigned index = 0; index < 4; ++index) {
    bytes[at + index] = static_cast<std::uint8_t>(value >> (index * 8U));
  }
}

std::uint32_t little(const std::span<const std::uint8_t> bytes, const std::size_t at) {
  std::uint32_t result = 0;
  for (unsigned index = 0; index < 4; ++index) {
    result |= std::uint32_t{bytes[at + index]} << (index * 8U);
  }
  return result;
}
}

MediaImage::MediaImage() {
  offsets_.fill(missing);
  block_offsets_.fill(missing);
}

MediaImage::MediaImage(std::vector<std::uint8_t> bytes) : bytes_(std::move(bytes)) {
  offsets_.fill(missing);
  block_offsets_.fill(missing);
  validate();
}

void MediaImage::validate() {
  if (bytes_.size() < header_bytes + 4 || bytes_.size() > max_bytes ||
      !std::ranges::equal(magic, std::span(bytes_).first(magic.size())) || little(bytes_, 8) != 1 ||
      little(bytes_, 12) != 1 || little(bytes_, page_size_offset) != page_bytes ||
      little(bytes_, reserved_offset) != 0) {
    throw std::runtime_error("Invalid media image header or length");
  }
  const auto page_count = little(bytes_, 16);
  const auto block_count = little(bytes_, block_count_offset);
  if (page_count > pages || block_count > pages / block_pages ||
      bytes_.size() != header_bytes + 4 + std::size_t{block_count} * 4 +
                           std::size_t{page_count} * record_bytes) {
    throw std::runtime_error("Media image record counts do not match its geometry");
  }
  static_assert(max_bytes <= std::numeric_limits<uInt>::max());
  const auto end = bytes_.size() - 4;
  if (little(bytes_, end) != crc32(0, bytes_.data(), static_cast<uInt>(end))) {
    throw std::runtime_error("Media image checksum mismatch");
  }
  std::size_t offset = header_bytes;
  std::uint32_t previous = missing;
  for (std::uint32_t index = 0; index < block_count; ++index, offset += 4) {
    const auto block = std::uint32_t{bytes_[offset]} | (std::uint32_t{bytes_[offset + 1]} << 8U);
    const auto known = bytes_[offset + 2];
    const auto value = bytes_[offset + 3];
    if (block >= pages / block_pages || (previous != missing && block <= previous) ||
        (known & ~7U) != 0 || (value & ~known) != 0) {
      throw std::runtime_error("Invalid media image block metadata");
    }
    block_offsets_[block] = static_cast<std::uint32_t>(offset);
    previous = block;
  }
  previous = missing;
  for (std::uint32_t index = 0; index < page_count; ++index, offset += record_bytes) {
    const auto page = little(bytes_, offset);
    const auto main_count = bytes_[offset + 4];
    const auto spare_count = bytes_[offset + 5];
    if (page >= pages || (previous != missing && page <= previous) ||
        ((main_count != all_bits || spare_count != all_bits) &&
         (main_count > 1 || spare_count > 2)) ||
        bytes_[offset + 6] != 0 || bytes_[offset + 7] != 0) {
      throw std::runtime_error("Invalid media image page metadata");
    }
    offsets_[page] = static_cast<std::uint32_t>(offset + 8);
    for (std::size_t column = 0; column < page_bytes; ++column) {
      const auto at = offset + 8 + (column * 2);
      if ((bytes_[at] & ~bytes_[at + 1]) != 0) {
        throw std::runtime_error("Noncanonical unknown bits in media image");
      }
    }
    previous = page;
  }
}

std::span<const std::uint8_t> MediaImage::page(const std::uint32_t index) const noexcept {
  if (index >= pages || offsets_[index] == missing) { return {}; }
  return std::span(bytes_).subspan(offsets_[index], std::size_t{page_bytes} * 2);
}

bool MediaImage::read(const Read access, std::uint8_t &value) const noexcept {
  const auto row = page(access.page);
  if (access.column >= page_bytes || row.empty()) { return false; }
  const auto at = std::size_t{access.column} * 2;
  value = row[at];
  return row[at + 1] == all_bits;
}

std::optional<bool> MediaImage::erase(const std::uint32_t block) {
  if (block >= block_offsets_.size() || block_offsets_[block] == missing) { return {}; }
  const auto metadata = block_offsets_[block];
  const auto known = bytes_[metadata + 2];
  const auto flags = bytes_[metadata + 3];
  if ((known & block_valid) != 0 && (flags & block_valid) == 0) { return false; }
  if ((known & erase_protected) != 0 && (flags & erase_protected) != 0) { return false; }
  if ((known & (block_valid | erase_protected)) != (block_valid | erase_protected)) { return {}; }
  const auto first = block * block_pages;
  unsigned absent = 0;
  for (unsigned page = first; page < first + block_pages; ++page) {
    if (offsets_[page] == missing) { ++absent; }
  }
  auto position = bytes_.size() - 4;
  for (unsigned page = first; page < pages; ++page) {
    if (offsets_[page] != missing) {
      position = offsets_[page] - 8;
      break;
    }
  }
  const auto added = std::size_t{absent} * record_bytes;
  if (added != 0) {
    bytes_.insert(bytes_.begin() + static_cast<std::ptrdiff_t>(position), added, 0);
    for (unsigned page = first; page < pages; ++page) {
      if (offsets_[page] != missing) { offsets_[page] += static_cast<std::uint32_t>(added); }
    }
    store(bytes_, page_count_offset, little(bytes_, page_count_offset) + absent);
  }
  for (unsigned index = 0; index < block_pages; ++index) {
    const auto record = position + (std::size_t{index} * record_bytes);
    store(bytes_, record, first + index);
    store(bytes_, record + 4, 0); // Known zero program counts and reserved fields.
    const auto data = record + 8;
    std::ranges::fill(std::span(bytes_).subspan(data, std::size_t{page_bytes} * 2), all_bits);
    offsets_[first + index] = static_cast<std::uint32_t>(data);
  }
  bytes_[metadata + 2] |= program_protected;
  bytes_[metadata + 3] &= static_cast<std::uint8_t>(~program_protected);
  checksum_dirty_ = true;
  return true;
}

void MediaImage::interrupt_erase(const std::uint32_t block) noexcept {
  if (bytes_.empty() || block >= block_offsets_.size()) { return; }
  const auto metadata = block_offsets_[block];
  if (metadata != missing) {
    const auto known = bytes_[metadata + 2];
    const auto flags = bytes_[metadata + 3];
    if (((known & block_valid) != 0 && (flags & block_valid) == 0) ||
        ((known & erase_protected) != 0 && (flags & erase_protected) != 0)) {
      return;
    }
  }
  const auto first = block * block_pages;
  for (unsigned page = first; page < first + block_pages; ++page) {
    if (offsets_[page] == missing) { continue; }
    const auto at = offsets_[page];
    bytes_[at - 4] = bytes_[at - 3] = all_bits;
    std::ranges::fill(std::span(bytes_).subspan(at, std::size_t{page_bytes} * 2), 0);
  }
  if (metadata != missing) {
    // Validity and erase protection survive; program protection may have cleared.
    bytes_[metadata + 2] &= static_cast<std::uint8_t>(~program_protected);
    bytes_[metadata + 3] &= static_cast<std::uint8_t>(~program_protected);
  }
  checksum_dirty_ = true;
}

std::span<const std::uint8_t> MediaImage::encoded() noexcept {
  if (checksum_dirty_) {
    const auto end = bytes_.size() - 4;
    store(bytes_, end, static_cast<std::uint32_t>(crc32(0, bytes_.data(), static_cast<uInt>(end))));
    checksum_dirty_ = false;
  }
  return bytes_;
}

}
