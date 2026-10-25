// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_cycle_cache.hpp"
#include "dsp_cycle_cache_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using Cache = blofeld::DspCycleCache;
constexpr std::size_t boot = 0xff0000, last = 0xffffff;
constexpr std::size_t maximum_storage = std::size_t{1024} * 1024;
constexpr std::uint8_t maximum_cycles = 255;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <typename Error> void rejects(const auto &operation) {
  try {
    operation();
  } catch (const Error &) { return; }
  throw std::runtime_error("Expected cycle-cache rejection");
}

void sparse_space_and_invalidation() {
  Cache cache;
  cache.reset(Cache::maximum_words);
  const auto empty_bytes = cache.storage_bytes();
  require(cache.size() == Cache::maximum_words && !cache.empty() && cache.resident_pages() == 0 &&
              empty_bytes < maximum_storage,
          "All 24 address bits retain zero cycles without a dense 16MiB byte vector");
  for (std::size_t at = 0; at < Cache::maximum_words; at += Cache::page_words) {
    require(cache.read(at) == 0, "Every cold page must return the zero memoization sentinel");
    cache.invalidate(at);
  }
  require(cache.resident_pages() == 0 && cache.storage_bytes() == empty_bytes,
          "Cold reads and code invalidation allocate no pages");
  cache[0] = 1;
  auto *saved = &cache[0];
  cache[Cache::page_words - 1] = 2;
  cache[Cache::page_words] = 3;
  cache[boot] = 4;
  cache[last] = maximum_cycles;
  require(cache.resident_pages() == 4 && cache.storage_bytes() < maximum_storage &&
              &cache[0] == saved && *saved == 1 && cache.read(last) == maximum_cycles,
          "Low/high/boundary pages preserve values and references as other pages are added");
  cache.invalidate(Cache::page_words - 1);
  require(cache.read(Cache::page_words - 1) == 0 && cache.read(0) == 1 &&
              cache.read(Cache::page_words) == 3 && cache.resident_pages() == 4,
          "One invalidation clears exactly its byte and retains neighboring cycle counts");
  rejects<std::out_of_range>([&] { static_cast<void>(cache.read(Cache::maximum_words)); });
  rejects<std::out_of_range>([&] { static_cast<void>(cache[Cache::maximum_words]); });
  rejects<std::out_of_range>([&] { cache.invalidate(Cache::maximum_words); });
  rejects<std::length_error>([&] { cache.reset(Cache::maximum_words + 1); });
  require(cache.size() == Cache::maximum_words && cache.read(last) == maximum_cycles &&
              cache.resident_pages() == 4,
          "Rejected bounds and resets preserve all prior cache state");
  std::cout << "Cycle-cache storage bytes: cold=" << empty_bytes
            << " four_pages=" << cache.storage_bytes() << '\n';
  cache.reset(Cache::page_words + 1);
  require(cache.resident_pages() == 0 && cache.read(Cache::page_words) == 0,
          "Reset clears memoized values and retains a partial last-page bound");
  rejects<std::out_of_range>([&] { static_cast<void>(cache[Cache::page_words + 1]); });
  cache.reset(0);
  require(cache.empty() && cache.resident_pages() == 0 && cache.storage_bytes() == 0,
          "Zero-size reset releases directory and resident pages");
}

void copies_moves_and_dense_expectations() {
  constexpr std::size_t count = (3 * Cache::page_words) + 1;
  Cache cache;
  cache.reset(count);
  std::vector<std::uint8_t> expected(count);
  for (std::size_t at = 0; at < count; at += 7) {
    expected[at] = static_cast<std::uint8_t>((at % maximum_cycles) + 1);
    cache[at] = expected[at];
  }
  Cache saved(cache);
  Cache assigned;
  assigned.reset(1);
  assigned[0] = maximum_cycles;
  assigned = cache;
  for (std::size_t at = 0; at < count; at += 3) { cache.invalidate(at); }
  for (std::size_t at = 0; at < count; ++at) {
    const auto original = at % 3 == 0 ? 0 : expected[at];
    require(
        cache.read(at) == original && saved.read(at) == expected[at] &&
            assigned.read(at) == expected[at],
        "Snapshot copies retain every warm and cold value independently of later invalidations");
  }
  saved[1] = 8;
  require(cache.read(1) == 0 && assigned.read(1) == 0,
          "Writing a snapshot branch cannot alias either the source or another copy");
  Cache moved(std::move(assigned));
  assigned = Cache{};
  assigned.reset(1);
  assigned[0] = 8;
  require(assigned.read(0) == 8, "A moved-from cache supports reset and reuse");
  assigned = std::move(moved);
  moved = Cache{};
  require(moved.empty() && moved.resident_pages() == 0 && assigned.size() == count,
          "Move assignment transfers the full bound and replaces the prior destination page");
  for (std::size_t at = 0; at < count; ++at) {
    require(assigned.read(at) == expected[at], "Moves preserve all original memoized values");
  }
  rejects<std::out_of_range>([&] { static_cast<void>(moved.read(0)); });
  cache.reset(count);
  for (std::size_t at = 0; at < count; ++at) {
    require(cache.read(at) == 0, "A complete reset restores every implicit zero value");
  }
}
}

int main() {
  try {
    sparse_space_and_invalidation();
    copies_moves_and_dense_expectations();
    blofeld::test::native_cycle_cache();
    std::cout
        << "Sparse cycle memoization retains all 24 address bits, values and snapshot isolation.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
