// SPDX-License-Identifier: GPL-3.0-only
#include "storage/flash_image.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld {
namespace {
constexpr std::array<std::uint8_t, 8> magic{'B', 'L', 'F', 'N', 'V', 'M', '\r', '\n'};
constexpr std::uint32_t version = 1;
constexpr unsigned erased = 0xff;
constexpr std::size_t version_offset = 8;
constexpr std::size_t length_offset = 12;
constexpr std::size_t checksum_offset = flash_image_file_bytes - sizeof(std::uint32_t);

void append_u32(std::vector<std::uint8_t> &output, const std::uint32_t value) {
  for (unsigned shift = 0; shift < 32; shift += 8) {
    output.push_back(static_cast<std::uint8_t>((value >> shift) & erased));
  }
}

std::uint32_t read_u32(const std::span<const std::uint8_t> bytes, const std::size_t offset) {
  std::uint32_t value = 0;
  for (unsigned index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(bytes[offset + index]) << (index * 8U);
  }
  return value;
}

std::uint32_t checksum(const std::span<const std::uint8_t> bytes) {
  return static_cast<std::uint32_t>(crc32(0, bytes.data(), static_cast<uInt>(bytes.size())));
}

template <std::size_t Count>
void append_bits(std::vector<std::uint8_t> &output, const std::bitset<Count> &bits) {
  for (std::size_t offset = 0; offset < Count; offset += 8) {
    unsigned value = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
      if (bits.test(offset + bit)) { value |= 1U << bit; }
    }
    output.push_back(static_cast<std::uint8_t>(value));
  }
}

template <std::size_t Count>
std::bitset<Count> read_bits(const std::span<const std::uint8_t> bytes) {
  std::bitset<Count> bits;
  for (std::size_t index = 0; index < Count; ++index) {
    bits.set(index, (bytes[index / 8] & (1U << (index % 8))) != 0);
  }
  return bits;
}
}

McuFlashImage McuFlashImage::from_known(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() != byte_count) { throw Fault("MCU NVM input requires exactly 32768 bytes"); }
  McuFlashImage image;
  std::ranges::copy(bytes, image.bytes.begin());
  image.known.set();
  return image;
}

void McuFlashImage::validate() const {
  for (std::size_t offset = 0; offset < byte_count; ++offset) {
    if (!known.test(offset) && bytes[offset] != 0) {
      throw Fault("Noncanonical unknown MCU NVM byte at physical " + hex(offset));
    }
  }
  for (std::size_t word = 0; word < word_count; ++word) {
    if (!fresh.test(word)) { continue; }
    const auto offset = word * 2;
    if (!known.test(offset) || !known.test(offset + 1) || bytes[offset] != erased ||
        bytes[offset + 1] != erased) {
      throw Fault("Inconsistent MCU NVM erase history at physical " + hex(offset));
    }
  }
}

void McuFlashImage::invalidate(const FlashRange range) {
  if (range.size == 0 || range.offset >= byte_count || range.size > byte_count - range.offset) {
    throw Fault("Invalid MCU NVM image range at physical " + hex(range.offset));
  }
  for (unsigned offset = range.offset; offset < range.offset + range.size; ++offset) {
    bytes[offset] = 0;
    known.reset(offset);
    fresh.reset(offset / 2);
  }
}

std::vector<std::uint8_t> encode_flash_image(const McuFlashImage &image) {
  image.validate();
  std::vector<std::uint8_t> bytes;
  bytes.reserve(flash_image_file_bytes);
  bytes.insert(bytes.end(), magic.begin(), magic.end());
  append_u32(bytes, version);
  append_u32(bytes, static_cast<std::uint32_t>(flash_image_payload_bytes));
  bytes.insert(bytes.end(), image.bytes.begin(), image.bytes.end());
  append_bits(bytes, image.known);
  append_bits(bytes, image.fresh);
  append_u32(bytes, checksum(bytes));
  return bytes;
}

McuFlashImage decode_flash_image(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() != flash_image_file_bytes) { throw Fault("Invalid MCU NVM file length"); }
  if (!std::ranges::equal(magic, bytes.first(magic.size())) ||
      read_u32(bytes, version_offset) != version ||
      read_u32(bytes, length_offset) != flash_image_payload_bytes) {
    throw Fault("Unsupported MCU NVM file header or version");
  }
  if (read_u32(bytes, checksum_offset) != checksum(bytes.first(checksum_offset))) {
    throw Fault("MCU NVM file checksum mismatch");
  }
  McuFlashImage image;
  std::ranges::copy(bytes.subspan(flash_image_header_bytes, image.bytes.size()),
                    image.bytes.begin());
  constexpr auto known_offset = flash_image_header_bytes + McuFlashImage::byte_count;
  constexpr auto fresh_offset = known_offset + flash_image_known_bytes;
  image.known =
      read_bits<McuFlashImage::byte_count>(bytes.subspan(known_offset, flash_image_known_bytes));
  image.fresh =
      read_bits<McuFlashImage::word_count>(bytes.subspan(fresh_offset, flash_image_fresh_bytes));
  image.validate();
  return image;
}

}
