// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace blofeld {

// Host cycle-count memoization. Absent pages contain zero; snapshots own deep copies.
// First use allocates, so neither this cache nor full checkpoint operations are real-time APIs.
class DspCycleCache {
public:
  static constexpr std::size_t maximum_words = std::size_t{1} << 24;
  static constexpr std::size_t page_words = 1024;

  DspCycleCache() = default;
  ~DspCycleCache() = default;

  DspCycleCache(const DspCycleCache &other)
      : pages_(other.pages_.size()), words_(other.words_), resident_(other.resident_) {
    for (std::size_t i = 0; i < pages_.size(); ++i) {
      if (other.pages_[i]) { pages_[i] = std::make_unique<Page>(*other.pages_[i]); }
    }
  }

  DspCycleCache(DspCycleCache &&other) noexcept { swap(other); }

  DspCycleCache &operator=(const DspCycleCache &other) {
    if (this != &other) {
      DspCycleCache copy(other);
      swap(copy);
    }
    return *this;
  }

  DspCycleCache &operator=(DspCycleCache &&other) noexcept {
    if (this != &other) {
      DspCycleCache moved(std::move(other));
      swap(moved);
    }
    return *this;
  }

  void reset(const std::size_t words) {
    if (words > maximum_words) { throw std::length_error("DSP cycle cache exceeds 24-bit space"); }
    DspCycleCache empty;
    empty.pages_.resize((words / page_words) + static_cast<std::size_t>(words % page_words != 0));
    empty.words_ = words;
    swap(empty);
  }

  [[nodiscard]] std::size_t size() const noexcept { return words_; }

  [[nodiscard]] bool empty() const noexcept { return words_ == 0; }

  [[nodiscard]] std::size_t resident_pages() const noexcept { return resident_; }

  [[nodiscard]] std::size_t storage_bytes() const noexcept {
    return (pages_.capacity() * sizeof(std::unique_ptr<Page>)) + (resident_ * sizeof(Page));
  }

  [[nodiscard]] std::uint8_t read(const std::size_t address) const {
    check_address(address);
    const auto &page = pages_[address / page_words];
    if (!page) { return 0; }
    return (*page)[address % page_words];
  }

  // Clearing a cold entry must not materialize a zero-filled page.
  void invalidate(const std::size_t address) {
    check_address(address);
    const auto &page = pages_[address / page_words];
    if (page) { (*page)[address % page_words] = 0; }
  }

  [[nodiscard]] std::uint8_t &operator[](const std::size_t address) {
    check_address(address);
    auto &page = pages_[address / page_words];
    if (!page) {
      page = std::make_unique<Page>();
      ++resident_;
    }
    return (*page)[address % page_words];
  }

  void swap(DspCycleCache &other) noexcept {
    pages_.swap(other.pages_);
    std::swap(words_, other.words_);
    std::swap(resident_, other.resident_);
  }

private:
  void check_address(const std::size_t address) const {
    if (address >= words_) { throw std::out_of_range("DSP cycle cache address outside space"); }
  }

  using Page = std::array<std::uint8_t, page_words>;
  std::vector<std::unique_ptr<Page>> pages_;
  std::size_t words_ = 0;
  std::size_t resident_ = 0;
};
}
