// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::vi::update {
struct Block {
  unsigned mode = 0;
  std::uint32_t address = 0;
  std::size_t declared_bytes = 0;
  // Original complete big-endian transport words, including supplied trailing words.
  std::vector<std::uint8_t> bytes;
};

[[nodiscard]] std::vector<Block> decode_midi(std::span<const std::uint8_t> midi);
}
