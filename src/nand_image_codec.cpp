// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld {
namespace {
constexpr std::array<std::uint8_t, 8> magic{'B', 'L', 'F', 'N', 'A', 'N', 'D', '\n'};
constexpr unsigned version = 1;
constexpr unsigned geometry = 1;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned block_flag_mask = 7;
enum class Width : std::uint8_t { byte = 1, half = 2, word = 4 };

struct Counts {
  unsigned pages;
  unsigned blocks;
};

constexpr std::size_t version_offset = 8;
constexpr std::size_t geometry_offset = 12;
constexpr std::size_t page_count_offset = 16;
constexpr std::size_t block_count_offset = 20;
constexpr std::size_t page_size_offset = 24;
constexpr std::size_t reserved_offset = 28;

void append(std::vector<std::uint8_t> &output,
            const std::uint32_t value,
            const Width width = Width::word) {
  for (unsigned index = 0; index < static_cast<unsigned>(width); ++index) {
    output.push_back(static_cast<std::uint8_t>((value >> (index * 8U)) & byte_mask));
  }
}

std::uint32_t little(const std::span<const std::uint8_t> bytes,
                     const std::size_t offset,
                     const Width width = Width::word) {
  std::uint32_t result = 0;
  for (unsigned index = 0; index < static_cast<unsigned>(width); ++index) {
    result |= static_cast<std::uint32_t>(bytes[offset + index]) << (index * 8U);
  }
  return result;
}

std::uint32_t checksum(const std::span<const std::uint8_t> bytes) {
  static_assert(nand_image_max_bytes <= std::numeric_limits<uInt>::max());
  return static_cast<std::uint32_t>(crc32(0, bytes.data(), static_cast<uInt>(bytes.size())));
}

unsigned encode_flags(const NandBlock block, const bool values) {
  unsigned result = 0;
  const std::array fields{block.valid, block.program_protected, block.erase_protected};
  for (unsigned index = 0; index < fields.size(); ++index) {
    const auto field = fields.at(index);
    if (field.has_value() && (!values || *field)) { result |= 1U << index; }
  }
  return result;
}

std::optional<bool> flag(const unsigned known, const unsigned values, const unsigned mask) {
  return (known & mask) != 0 ? std::optional<bool>((values & mask) != 0) : std::nullopt;
}

NandBlock decode_flags(const unsigned known, const unsigned values) {
  if ((known & ~block_flag_mask) != 0 || (values & ~known) != 0) {
    throw Fault("Noncanonical NAND image block flags");
  }
  return {.valid = flag(known, values, 1),
          .program_protected = flag(known, values, 2),
          .erase_protected = flag(known, values, 4)};
}

NandArray::Page decode_page(const std::span<const std::uint8_t> bytes) {
  NandArray::Page page;
  const auto main = bytes[4];
  const auto spare = bytes[5];
  if (main != byte_mask || spare != byte_mask) {
    if (main > 1 || spare > 2) { throw Fault("Invalid NAND image program counts"); }
    page.programs = {.main = main, .spare = spare};
  }
  if (little(bytes, 6, Width::half) != 0) { throw Fault("Nonzero NAND image page reserved field"); }
  for (std::size_t column = 0; column < NandArray::page_bytes; ++column) {
    constexpr std::size_t cell_offset = 8;
    const auto offset = cell_offset + (column * 2);
    page.cells.at(column) = {.value = bytes[offset], .known = bytes[offset + 1]};
  }
  return page;
}

void decode_records(NandImage &image,
                    const std::span<const std::uint8_t> bytes,
                    const Counts counts) {
  std::size_t offset = nand_image_header_bytes;
  std::optional<unsigned> previous;
  for (unsigned index = 0; index < counts.blocks; ++index) {
    const auto block = little(bytes, offset, Width::half);
    if (block >= NandArray::blocks || (previous && block <= *previous)) {
      throw Fault("Invalid or unordered NAND image block index");
    }
    image.blocks.emplace(block, decode_flags(bytes[offset + 2], bytes[offset + 3]));
    previous = block;
    offset += nand_image_block_bytes;
  }
  previous.reset();
  for (unsigned index = 0; index < counts.pages; ++index) {
    const auto page = little(bytes, offset);
    if (page >= NandArray::pages || (previous && page <= *previous)) {
      throw Fault("Invalid or unordered NAND image page index");
    }
    image.pages.emplace(page, decode_page(bytes.subspan(offset, nand_image_page_bytes)));
    previous = page;
    offset += nand_image_page_bytes;
  }
}
}

std::vector<std::uint8_t> encode_nand_image(const NandImage &image) {
  image.validate();
  std::vector<std::uint8_t> result;
  result.reserve(nand_image_header_bytes + (image.blocks.size() * nand_image_block_bytes) +
                 (image.pages.size() * nand_image_page_bytes) + sizeof(std::uint32_t));
  result.insert(result.end(), magic.begin(), magic.end());
  append(result, version);
  append(result, geometry);
  append(result, static_cast<std::uint32_t>(image.pages.size()));
  append(result, static_cast<std::uint32_t>(image.blocks.size()));
  append(result, NandArray::page_bytes);
  append(result, 0);
  for (const auto &[index, block] : image.blocks) {
    append(result, index, Width::half);
    append(result, encode_flags(block, false), Width::byte);
    append(result, encode_flags(block, true), Width::byte);
  }
  for (const auto &[index, page] : image.pages) {
    append(result, index);
    append(result, page.programs ? page.programs->main : byte_mask, Width::byte);
    append(result, page.programs ? page.programs->spare : byte_mask, Width::byte);
    append(result, 0, Width::half);
    for (const auto cell : page.cells) {
      result.push_back(cell.value);
      result.push_back(cell.known);
    }
  }
  append(result, checksum(result));
  return result;
}

NandImage decode_nand_image(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() < nand_image_header_bytes + sizeof(std::uint32_t) ||
      bytes.size() > nand_image_max_bytes) {
    throw Fault("Invalid NAND image file length");
  }
  if (!std::ranges::equal(magic, bytes.first(magic.size())) ||
      little(bytes, version_offset) != version || little(bytes, geometry_offset) != geometry ||
      little(bytes, page_size_offset) != NandArray::page_bytes ||
      little(bytes, reserved_offset) != 0) {
    throw Fault("Unsupported NAND image header, geometry or version");
  }
  const auto pages = little(bytes, page_count_offset);
  const auto blocks = little(bytes, block_count_offset);
  if (pages > NandArray::pages || blocks > NandArray::blocks) {
    throw Fault("NAND image record count exceeds geometry");
  }
  const auto expected = nand_image_header_bytes + sizeof(std::uint32_t) +
                        (static_cast<std::size_t>(blocks) * nand_image_block_bytes) +
                        (static_cast<std::size_t>(pages) * nand_image_page_bytes);
  if (bytes.size() != expected) {
    throw Fault("NAND image record counts do not match file length");
  }
  const auto end = bytes.size() - sizeof(std::uint32_t);
  if (little(bytes, end) != checksum(bytes.first(end))) {
    throw Fault("NAND image checksum mismatch");
  }
  NandImage image;
  decode_records(image, bytes, {.pages = pages, .blocks = blocks});
  image.validate();
  return image;
}
}
