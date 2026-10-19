// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <span>

namespace blofeld {

struct NandByte {
  std::uint8_t value = 0;
  std::uint8_t known = 0;
  bool operator==(const NandByte &) const = default;
};

struct NandLocation {
  unsigned page = 0;
  unsigned column = 0;
};

struct NandProgramCount {
  unsigned main = 0;
  unsigned spare = 0;
};

struct NandBlock {
  std::optional<bool> valid = std::nullopt;
  std::optional<bool> program_protected = std::nullopt;
  std::optional<bool> erase_protected = std::nullopt;
};

// Sparse cells and reference history, not an erased factory image or a file codec.
class NandArray {
public:
  static constexpr unsigned page_bytes = 528;
  static constexpr unsigned main_bytes = 512;
  static constexpr unsigned pages = 131072;
  static constexpr unsigned block_pages = 32;
  static constexpr unsigned blocks = pages / block_pages;

  struct Page {
    std::array<NandByte, page_bytes> cells{};
    std::optional<NandProgramCount> programs;
  };

  void observe_bytes(NandLocation start, std::span<const std::uint8_t> bytes);
  void observe_cell(NandLocation location, NandByte cell);
  void observe_block(unsigned block, NandBlock state);
  [[nodiscard]] NandByte cell(NandLocation location) const;
  [[nodiscard]] Page snapshot(unsigned page) const;
  [[nodiscard]] NandBlock block(unsigned block) const;
  static NandByte programmed(NandByte before, std::uint8_t data);
  void restore(unsigned page, Page state);
  void invalidate_page(unsigned page);
  void invalidate_block(unsigned block);
  void erase_block(unsigned block);

private:
  static void validate_location(NandLocation location);
  static void validate_page(unsigned page);
  static void validate_block(unsigned block);
  std::map<unsigned, Page> contents_;
  std::map<unsigned, NandBlock> blocks_;
};
}
