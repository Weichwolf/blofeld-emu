// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/nand_array.hpp"

#include "blofeld/bus.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld {
namespace {
constexpr unsigned byte_mask = 0xff;
}

void NandArray::validate_page(const unsigned page) {
  if (page >= pages) { throw Fault("NAND array page exceeds device: page=" + hex(page, 6)); }
}

void NandArray::validate_block(const unsigned block) {
  if (block >= blocks) { throw Fault("NAND array block exceeds device: block=" + hex(block, 4)); }
}

void NandArray::validate_location(const NandLocation location) {
  validate_page(location.page);
  if (location.column >= page_bytes) {
    throw Fault("NAND array column exceeds page: page=" + hex(location.page, 6) +
                " column=" + hex(location.column, 3));
  }
}

void NandArray::observe_bytes(const NandLocation start, const std::span<const std::uint8_t> bytes) {
  validate_page(start.page);
  if (start.column > page_bytes || bytes.size() > page_bytes - start.column) {
    throw Fault("NAND contents observation range exceeds page: page=" + hex(start.page, 6) +
                " column=" + hex(start.column, 3));
  }
  if (bytes.empty()) { return; }
  auto &target = contents_[start.page];
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    target.cells.at(start.column + index) = {.value = bytes[index], .known = byte_mask};
  }
}

void NandArray::observe_cell(const NandLocation location, const NandByte cell) {
  validate_location(location);
  if ((cell.value & ~cell.known) != 0) {
    throw Fault("Noncanonical NAND cell observation: page=" + hex(location.page, 6) +
                " column=" + hex(location.column, 3));
  }
  contents_[location.page].cells.at(location.column) = cell;
}

void NandArray::observe_block(const unsigned block, const NandBlock state) {
  validate_block(block);
  constexpr unsigned protection_blocks = 200;
  if (block >= protection_blocks &&
      (state.program_protected == true || state.erase_protected == true)) {
    throw Fault("NAND protected-block observation exceeds supported range: block=" + hex(block, 4));
  }
  blocks_[block] = state;
}

NandByte NandArray::cell(const NandLocation location) const {
  validate_location(location);
  const auto found = contents_.find(location.page);
  return found == contents_.end() ? NandByte{} : found->second.cells.at(location.column);
}

NandArray::Page NandArray::snapshot(const unsigned page) const {
  validate_page(page);
  const auto found = contents_.find(page);
  return found == contents_.end() ? Page{} : found->second;
}

NandBlock NandArray::block(const unsigned block) const {
  validate_block(block);
  const auto found = blocks_.find(block);
  return found == blocks_.end() ? NandBlock{} : found->second;
}

NandByte NandArray::programmed(const NandByte before, const std::uint8_t data) {
  return {.value = static_cast<std::uint8_t>(before.value & data),
          .known = static_cast<std::uint8_t>(before.known | (byte_mask ^ data))};
}

void NandArray::restore(const unsigned page, Page state) {
  validate_page(page);
  contents_[page] = state;
}

void NandArray::invalidate_page(const unsigned page) {
  validate_page(page);
  contents_[page] = {};
}

void NandArray::invalidate_block(const unsigned block) {
  validate_block(block);
  for (unsigned index = 0; index < block_pages; ++index) {
    invalidate_page((block * block_pages) + index);
  }
  blocks_[block] = {};
}

void NandArray::erase_block(const unsigned block) {
  validate_block(block);
  Page erased;
  erased.cells.fill({.value = byte_mask, .known = byte_mask});
  erased.programs = NandProgramCount{};
  for (unsigned index = 0; index < block_pages; ++index) {
    restore((block * block_pages) + index, erased);
  }
  blocks_[block] = {.valid = true, .program_protected = false, .erase_protected = false};
}
}
