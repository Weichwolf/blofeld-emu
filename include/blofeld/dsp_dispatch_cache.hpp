// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace blofeld {

// Host decoder memoization, independent of guest memory and physical instruction caches.
// First access allocates a page; references survive other page allocations, until reset.
template <typename Entry> class DspDispatchCache {
  static_assert(std::is_nothrow_copy_constructible_v<Entry>);
  static_assert(std::is_nothrow_copy_assignable_v<Entry>);

public:
  static constexpr std::size_t address_bits = 24;
  static constexpr std::size_t maximum_words = std::size_t{1} << address_bits;
  static constexpr std::size_t page_words = 1024;

  // Reset replaces all memoized entries. An invalid size leaves the old cache intact.
  void reset(const std::size_t words, const Entry &initial) {
    if (words > maximum_words) {
      throw std::length_error("DSP dispatch cache exceeds 24-bit space");
    }
    const auto count = (words / page_words) + static_cast<std::size_t>(words % page_words != 0);
    std::vector<std::unique_ptr<Page>> empty(count);
    pages_.swap(empty);
    initial_ = initial;
    words_ = words;
    resident_ = 0;
  }

  [[nodiscard]] std::size_t size() const noexcept { return words_; }

  [[nodiscard]] std::size_t resident_pages() const noexcept { return resident_; }

  // Container capacity only; allocator metadata is outside this diagnostic count.
  [[nodiscard]] std::size_t storage_bytes() const noexcept {
    return (pages_.capacity() * sizeof(std::unique_ptr<Page>)) + (resident_ * sizeof(Page));
  }

  // Invalidation of a cold address must not allocate a decoder page.
  [[nodiscard]] Entry *find(const std::size_t address) noexcept {
    if (address >= words_) { return nullptr; }
    const auto &page = pages_[address / page_words];
    return page ? &(*page)[address % page_words] : nullptr;
  }

  [[nodiscard]] Entry &at(const std::size_t address) {
    if (address >= words_) { throw std::out_of_range("DSP dispatch cache address outside space"); }
    auto &page = pages_[address / page_words];
    if (!page) {
      auto fresh = std::make_unique<Page>();
      fresh->fill(initial_);
      page = std::move(fresh);
      ++resident_;
    }
    return (*page)[address % page_words];
  }

  [[nodiscard]] Entry &operator[](const std::size_t address) { return at(address); }

private:
  using Page = std::array<Entry, page_words>;
  std::vector<std::unique_ptr<Page>> pages_;
  Entry initial_{};
  std::size_t words_ = 0;
  std::size_t resident_ = 0;
};
}
