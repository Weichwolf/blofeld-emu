// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_array.hpp"

#include "common/bus.hpp"
#include "storage/nand_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned byte_mask = 0xff;
constexpr unsigned group_pages = 512;
}

struct NandArray::Data {
  using Group = std::array<std::shared_ptr<Page>, group_pages>;
  using Blocks = std::array<std::optional<NandBlock>, blocks>;
  std::array<std::shared_ptr<Group>, pages / group_pages> groups;
  std::shared_ptr<Blocks> metadata;
};

NandArray::NandArray(const NandImage &image) {
  image.validate();
  for (const auto &[page, contents] : image.pages) { restore(page, contents); }
  for (const auto &[block, metadata] : image.blocks) { observe_block(block, metadata); }
}

NandArray::NandArray(const NandArray &other) : data_(other.data_) {}

NandArray &NandArray::operator=(const NandArray &other) {
  if (this != &other) {
    clear_read_cache();
    data_ = other.data_;
  }
  return *this;
}

NandArray::NandArray(NandArray &&other) noexcept : data_(std::move(other.data_)) {
  other.clear_read_cache();
}

NandArray &NandArray::operator=(NandArray &&other) noexcept {
  if (this != &other) {
    clear_read_cache();
    other.clear_read_cache();
    data_ = std::move(other.data_);
  }
  return *this;
}

const NandArray::Page *NandArray::find_page(const unsigned page) const {
  if (!data_) { return nullptr; }
  const auto &group = data_->groups.at(page / group_pages);
  return group ? group->at(page % group_pages).get() : nullptr;
}

void NandArray::writable_data() {
  if (!data_) {
    data_ = std::make_shared<Data>();
  } else if (!data_.unique()) {
    data_ = std::make_shared<Data>(*data_);
  }
}

NandArray::Page &NandArray::writable_page(const unsigned page) {
  clear_read_cache();
  writable_data();
  auto &group = data_->groups.at(page / group_pages);
  if (!group) {
    group = std::make_shared<Data::Group>();
  } else if (!group.unique()) {
    group = std::make_shared<Data::Group>(*group);
  }
  auto &contents = group->at(page % group_pages);
  if (!contents) {
    contents = std::make_shared<Page>();
  } else if (!contents.unique()) {
    contents = std::make_shared<Page>(*contents);
  }
  return *contents;
}

NandImage NandArray::image() const {
  NandImage result;
  if (!data_) { return result; }
  for (unsigned group = 0; group < data_->groups.size(); ++group) {
    const auto &contents = data_->groups.at(group);
    if (!contents) { continue; }
    for (unsigned offset = 0; offset < group_pages; ++offset) {
      if (contents->at(offset)) {
        result.pages.emplace((group * group_pages) + offset, *contents->at(offset));
      }
    }
  }
  if (data_->metadata) {
    for (unsigned block = 0; block < blocks; ++block) {
      const auto &entry = data_->metadata->at(block);
      if (entry) { result.blocks.emplace(block, *entry); }
    }
  }
  result.validate();
  return result;
}

NandImage NandArray::image(const NandRegion region) const {
  if (region.first_page >= pages || region.page_count == 0 ||
      region.page_count > pages - region.first_page) {
    throw Fault("Invalid persistent image region");
  }
  NandImage result;
  for (unsigned index = 0; index < region.page_count; ++index) {
    const auto page = region.first_page + index;
    result.pages.emplace(page, snapshot(page));
    const auto block_index = page / block_pages;
    result.blocks.try_emplace(block_index, block(block_index));
  }
  return result;
}

void NandArray::clear_read_cache() noexcept {
  read_page_ = nullptr;
  read_index_ = pages;
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
  auto &target = writable_page(start.page);
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
  writable_page(location.page).cells.at(location.column) = cell;
}

void NandArray::observe_block(const unsigned block, const NandBlock state) {
  validate_block(block);
  constexpr unsigned protection_blocks = 200;
  if (block >= protection_blocks &&
      (state.program_protected == true || state.erase_protected == true)) {
    throw Fault("NAND protected-block observation exceeds supported range: block=" + hex(block, 4));
  }
  writable_data();
  if (!data_->metadata) {
    data_->metadata = std::make_shared<Data::Blocks>();
  } else if (!data_->metadata.unique()) {
    data_->metadata = std::make_shared<Data::Blocks>(*data_->metadata);
  }
  data_->metadata->at(block) = state;
}

NandByte NandArray::cell(const NandLocation location) const {
  validate_location(location);
  if (read_index_ == location.page) { return read_page_->cells.at(location.column); }
  read_page_ = find_page(location.page);
  if (read_page_ == nullptr) { return {}; }
  read_index_ = location.page;
  return read_page_->cells.at(location.column);
}

NandArray::Page NandArray::snapshot(const unsigned page) const {
  validate_page(page);
  const auto *const contents = find_page(page);
  return contents == nullptr ? Page{} : *contents;
}

NandBlock NandArray::block(const unsigned block) const {
  validate_block(block);
  return data_ && data_->metadata ? data_->metadata->at(block).value_or(NandBlock{}) : NandBlock{};
}

NandByte NandArray::programmed(const NandByte before, const std::uint8_t data) {
  return {.value = static_cast<std::uint8_t>(before.value & data),
          .known = static_cast<std::uint8_t>(before.known | (byte_mask ^ data))};
}

void NandArray::restore(const unsigned page, Page state) {
  validate_page(page);
  writable_page(page) = state;
}

void NandArray::invalidate_page(const unsigned page) {
  validate_page(page);
  writable_page(page) = {};
}

void NandArray::invalidate_block(const unsigned block) {
  validate_block(block);
  for (unsigned index = 0; index < block_pages; ++index) {
    invalidate_page((block * block_pages) + index);
  }
  observe_block(block, {});
}

void NandArray::erase_block(const unsigned block) {
  validate_block(block);
  Page erased;
  erased.cells.fill({.value = byte_mask, .known = byte_mask});
  erased.programs = NandProgramCount{};
  for (unsigned index = 0; index < block_pages; ++index) {
    restore((block * block_pages) + index, erased);
  }
  observe_block(block, {.valid = true, .program_protected = false, .erase_protected = false});
}
}
