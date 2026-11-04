// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "memstate.hh"
#include "space.hh"
#include "types.h"

#include <cstddef>
#include <vector>

namespace blofeld {

struct PcodeStorageLayout {
  ghidra::AddrSpace *space;
  ghidra::int4 page_bytes;
  ghidra::int4 overflow_words;
  std::size_t dense_words;
};

// Internal SLEIGH register/unique storage only. Hardware accesses use BusBank.
// Preserve MemoryBank's byte packing and the original sparse hash fallback.
class PcodeStorage final : public ghidra::MemoryHashOverlay {
public:
  explicit PcodeStorage(const PcodeStorageLayout layout)
      : MemoryHashOverlay(
            layout.space, word_bytes, layout.page_bytes, layout.overflow_words, nullptr),
        dense_(layout.dense_words, 0) {}

private:
  void insert(const ghidra::uintb address, const ghidra::uintb value) override {
    const auto index = address / word_bytes;
    if (index < dense_.size()) {
      dense_[static_cast<std::size_t>(index)] = value;
      return;
    }
    MemoryHashOverlay::insert(address, value);
  }

  [[nodiscard]] ghidra::uintb find(const ghidra::uintb address) const override {
    const auto index = address / word_bytes;
    if (index < dense_.size()) { return dense_[static_cast<std::size_t>(index)]; }
    return MemoryHashOverlay::find(address);
  }

  static constexpr ghidra::int4 word_bytes = 8;
  std::vector<ghidra::uintb> dense_;
};

} // namespace blofeld
