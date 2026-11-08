// SPDX-License-Identifier: GPL-3.0-only
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld::vi {
namespace {
constexpr std::array<std::uint8_t, 8> magic{'B', 'L', 'F', 'N', 'V', 'M', '\r', '\n'};
constexpr std::size_t header_bytes = 16;
constexpr std::size_t known_offset = header_bytes + FlashImage::byte_count;
constexpr std::size_t erased_offset = known_offset + (FlashImage::byte_count / 8);
constexpr std::size_t checksum_offset = FlashImage::file_bytes - 4;
constexpr std::uint32_t byte_mask = 0xff;

std::uint32_t little(const std::span<const std::uint8_t> bytes, const std::size_t at) {
  std::uint32_t value = 0;
  for (unsigned index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(bytes[at + index]) << (index * 8U);
  }
  return value;
}

void store(const std::span<std::uint8_t> bytes, const std::size_t at, const std::uint32_t value) {
  for (unsigned index = 0; index < 4; ++index) {
    bytes[at + index] = static_cast<std::uint8_t>((value >> (index * 8U)) & byte_mask);
  }
}

std::uint32_t checksum(const std::span<const std::uint8_t> bytes) {
  return static_cast<std::uint32_t>(crc32(0, bytes.data(), static_cast<uInt>(bytes.size())));
}

void validate(const FlashImage &image) {
  for (std::size_t at = 0; at < FlashImage::byte_count; ++at) {
    if (image.known[at] > 1 || (image.known[at] == 0 && image.bytes[at] != 0)) {
      throw std::runtime_error("Noncanonical MCU flash knowledge at " + std::to_string(at));
    }
    const auto word = at / 2;
    if ((image.erased_words[word / 8] & (1U << (word % 8))) != 0 &&
        (image.known[at] == 0 || image.bytes[at] != byte_mask)) {
      throw std::runtime_error("Inconsistent MCU flash erase metadata at " + std::to_string(at));
    }
  }
}
}

FlashImage FlashImage::decode(const std::span<const std::uint8_t> file) {
  if (file.size() != file_bytes) { throw std::runtime_error("Invalid MCU image length"); }
  if (!std::ranges::equal(magic, file.first(magic.size())) || little(file, 8) != 1 ||
      little(file, 12) != checksum_offset - header_bytes) {
    throw std::runtime_error("Unsupported MCU image header or version");
  }
  if (little(file, checksum_offset) != checksum(file.first(checksum_offset))) {
    throw std::runtime_error("MCU image checksum mismatch");
  }
  FlashImage image;
  std::ranges::copy(file.subspan(header_bytes, byte_count), image.bytes.begin());
  for (std::size_t at = 0; at < byte_count; ++at) {
    image.known[at] = static_cast<std::uint8_t>((file[known_offset + (at / 8)] >> (at % 8)) & 1U);
  }
  std::ranges::copy(file.subspan(erased_offset, erase_map_bytes), image.erased_words.begin());
  validate(image);
  return image;
}

std::vector<std::uint8_t> FlashImage::encode() const {
  validate(*this);
  std::vector<std::uint8_t> file(file_bytes);
  std::ranges::copy(magic, file.begin());
  store(file, 8, 1);
  store(file, 12, static_cast<std::uint32_t>(checksum_offset - header_bytes));
  std::ranges::copy(bytes, file.begin() + header_bytes);
  for (std::size_t at = 0; at < byte_count; ++at) {
    file[known_offset + (at / 8)] |= static_cast<std::uint8_t>(known[at] << (at % 8));
  }
  std::ranges::copy(erased_words, file.begin() + erased_offset);
  store(file, checksum_offset, checksum(std::span(file).first(checksum_offset)));
  return file;
}
}
