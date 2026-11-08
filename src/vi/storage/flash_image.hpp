// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::vi {
// Persistent MCU bytes and knowledge. No CPU, RAM, clock or peripheral snapshot.
struct FlashImage {
  static constexpr std::size_t byte_count = 32768;
  static constexpr std::size_t erase_map_bytes = byte_count / 16;
  static constexpr std::size_t file_bytes =
      16 + byte_count + (byte_count / 8) + erase_map_bytes + 4;
  std::array<std::uint8_t, byte_count> bytes{};
  // One byte per location for direct CPU12 memory grants: zero unknown, one known.
  std::array<std::uint8_t, byte_count> known{};
  // Existing .bfnvm word-program eligibility, retained across image round trips.
  std::array<std::uint8_t, erase_map_bytes> erased_words{};

  [[nodiscard]] static FlashImage decode(std::span<const std::uint8_t> file);
  [[nodiscard]] std::vector<std::uint8_t> encode() const;
};
}
