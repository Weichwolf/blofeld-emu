// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld {

struct FlashRange {
  std::uint16_t offset = 0;
  unsigned size = 0;
};

// Persistent MCU NVM only. No CPU, RAM, controller queue or board snapshot.
struct McuFlashImage {
  static constexpr std::size_t byte_count = 32768;
  static constexpr std::size_t word_count = byte_count / 2;
  std::array<std::uint8_t, byte_count> bytes{};
  std::bitset<byte_count> known;
  std::bitset<word_count> fresh;

  static McuFlashImage from_known(std::span<const std::uint8_t> bytes);
  void validate() const;
  void invalidate(FlashRange range);
};

inline constexpr std::size_t flash_image_header_bytes = 16;
inline constexpr std::size_t flash_image_known_bytes = McuFlashImage::byte_count / 8;
inline constexpr std::size_t flash_image_fresh_bytes = McuFlashImage::word_count / 8;
inline constexpr std::size_t flash_image_payload_bytes =
    McuFlashImage::byte_count + flash_image_known_bytes + flash_image_fresh_bytes;
inline constexpr std::size_t flash_image_file_bytes =
    flash_image_header_bytes + flash_image_payload_bytes + sizeof(std::uint32_t);

[[nodiscard]] std::vector<std::uint8_t> encode_flash_image(const McuFlashImage &image);
[[nodiscard]] McuFlashImage decode_flash_image(std::span<const std::uint8_t> bytes);

}
