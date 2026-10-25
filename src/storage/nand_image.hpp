// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand_array.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <vector>

namespace blofeld {

// Physical NAND knowledge only. No CPU, protocol phase, WP/readiness observation
// or pending-operation preimage is restored from a persistent image.
struct NandImage {
  std::map<unsigned, NandArray::Page> pages;
  std::map<unsigned, NandBlock> blocks;
  void validate() const;
};

inline constexpr std::size_t nand_image_header_bytes = 32;
inline constexpr std::size_t nand_image_block_bytes = 4;
inline constexpr std::size_t nand_image_page_bytes = 8 + (NandArray::page_bytes * 2);
inline constexpr std::size_t nand_image_max_bytes =
    nand_image_header_bytes +
    (static_cast<std::size_t>(NandArray::blocks) * nand_image_block_bytes) +
    (static_cast<std::size_t>(NandArray::pages) * nand_image_page_bytes) + sizeof(std::uint32_t);

[[nodiscard]] std::vector<std::uint8_t> encode_nand_image(const NandImage &image);
[[nodiscard]] NandImage decode_nand_image(std::span<const std::uint8_t> bytes);

}
