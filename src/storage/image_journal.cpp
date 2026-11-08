// SPDX-License-Identifier: GPL-3.0-only
#include "storage/image_journal.hpp"

#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_image.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld {
namespace {
constexpr std::array<std::uint8_t, 8> magic{'B', 'F', 'D', 'E', 'L', 'T', '1', '\n'};
constexpr std::array<std::uint8_t, 8> committed{'C', 'O', 'M', 'M', 'I', 'T', '1', '\n'};
constexpr std::size_t header_bytes = 24;
constexpr std::size_t footer_bytes = 20;
constexpr std::size_t sequence_offset = 12;
constexpr std::size_t checksum_offset = 20;
constexpr std::size_t footer_checksum = 16;
constexpr std::size_t maximum_payload = nand_image_header_bytes + nand_image_block_bytes +
                                        (NandArray::block_pages * nand_image_page_bytes) + 4;
constexpr unsigned byte_mask = 0xff;
enum class Width : std::uint8_t { word = 4, wide = 8 };

std::uint64_t little(const std::span<const std::uint8_t> bytes,
                     const std::size_t offset,
                     const Width width = Width::word) {
  std::uint64_t result = 0;
  for (std::size_t i = 0; i < static_cast<unsigned>(width); ++i) {
    result |= static_cast<std::uint64_t>(bytes[offset + i]) << (i * 8U);
  }
  return result;
}

void append(std::vector<std::uint8_t> &bytes,
            const std::uint64_t value,
            const Width width = Width::word) {
  for (unsigned i = 0; i < static_cast<unsigned>(width); ++i) {
    bytes.push_back(static_cast<std::uint8_t>((value >> (i * 8U)) & byte_mask));
  }
}

std::uint32_t checksum(const std::span<const std::uint8_t> bytes, const std::uint32_t initial = 0) {
  return static_cast<std::uint32_t>(crc32(initial, bytes.data(), static_cast<uInt>(bytes.size())));
}

void merge(NandImage &image, const NandImage &changes) {
  for (const auto &[page, data] : changes.pages) { image.pages.insert_or_assign(page, data); }
  for (const auto &[block, data] : changes.blocks) { image.blocks.insert_or_assign(block, data); }
}

bool replay(const PrivateFile &file,
            const std::uint64_t end,
            ImageJournalPosition &position,
            NandImage &image) {
  if (end - position.offset < header_bytes) {
    std::array<std::uint8_t, header_bytes> partial{};
    const auto length = static_cast<std::size_t>(end - position.offset);
    file.read({.offset = position.offset, .bytes = std::span(partial).first(length)});
    const auto prefix = std::min(length, magic.size());
    if (!std::ranges::equal(std::span(partial).first(prefix), std::span(magic).first(prefix))) {
      throw Fault("Invalid image journal tail");
    }
    return false;
  }
  std::array<std::uint8_t, header_bytes> header{};
  file.read({.offset = position.offset, .bytes = header});
  if (!std::ranges::equal(magic, std::span(header).first(magic.size())) ||
      little(header, checksum_offset) != checksum(std::span(header).first(checksum_offset)) ||
      position.sequence == std::numeric_limits<std::uint64_t>::max() ||
      little(header, sequence_offset, Width::wide) != position.sequence + 1) {
    throw Fault("Invalid image journal header or sequence");
  }
  const auto length = little(header, magic.size());
  if (length < nand_image_header_bytes + 4 || length > maximum_payload) {
    throw Fault("Image journal payload exceeds a page/block transaction");
  }
  const auto total = header_bytes + length + footer_bytes;
  if (end - position.offset < total) { return false; }
  std::array<std::uint8_t, footer_bytes> footer{};
  file.read({.offset = position.offset + header_bytes + length, .bytes = footer});
  if (!std::ranges::equal(committed, std::span(footer).first(committed.size())) ||
      little(footer, committed.size(), Width::wide) != position.sequence + 1) {
    throw Fault("Invalid image journal commit marker");
  }
  std::vector<std::uint8_t> payload(static_cast<std::size_t>(length));
  file.read({.offset = position.offset + header_bytes, .bytes = payload});
  if (little(footer, footer_checksum) != checksum(payload, checksum(header))) {
    throw Fault("Image journal transaction checksum mismatch");
  }
  merge(image, decode_nand_image(payload));
  position.offset += total;
  ++position.sequence;
  return true;
}
}

NandImage load_image_journal(const PrivateFile &file, ImageJournalPosition &position) {
  const auto end = file.size();
  if (end < nand_image_header_bytes) { throw Fault("Incomplete NAND image file"); }
  std::array<std::uint8_t, nand_image_header_bytes> header{};
  file.read({.bytes = header});
  const auto length = nand_image_base_bytes(header);
  if (length > end) { throw Fault("Incomplete NAND image base"); }
  std::vector<std::uint8_t> base(length);
  file.read({.bytes = base});
  auto image = decode_nand_image(base);
  position = {.offset = length};
  while (position.offset < end && replay(file, end, position, image)) {}
  return image;
}

void append_image_journal(PrivateFile &file,
                          ImageJournalPosition &position,
                          const NandImage &changes,
                          const PrivateFileObserver &observer) {
  if (position.sequence == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Image journal sequence exhausted");
  }
  if (changes.pages.empty() || changes.pages.size() > NandArray::block_pages ||
      changes.blocks.size() > 1) {
    throw Fault("Image update exceeds a page/block transaction");
  }
  const auto payload = encode_nand_image(changes);
  if (payload.size() > maximum_payload) { throw Fault("Image update exceeds a page/block"); }
  std::vector<std::uint8_t> header(magic.begin(), magic.end());
  append(header, payload.size());
  append(header, position.sequence + 1, Width::wide);
  append(header, checksum(header));
  std::vector<std::uint8_t> footer(committed.begin(), committed.end());
  append(footer, position.sequence + 1, Width::wide);
  append(footer, checksum(payload, checksum(header)));
  auto pending = header;
  pending.insert(pending.end(), payload.begin(), payload.end());
  file.append({.offset = position.offset, .pending = pending, .commit = footer}, observer);
  position.offset += pending.size() + footer.size();
  ++position.sequence;
}
}
