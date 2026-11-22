// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/interrupts.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <optional>
#include <span>

namespace blofeld::vi::dsp {
namespace {
using dsp56300::Word;
// Table 4-5 order breaks ties within an IPL; numeric vector order does not.
constexpr std::array<Word, 43> order{
    0x10, 0x12, 0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e, 0x20, 0x22, 0x34, 0x32, 0x30, 0x36, 0x3c,
    0x3e, 0x3a, 0x38, 0x4c, 0x4a, 0x42, 0x48, 0x40, 0x44, 0x28, 0x2a, 0x2e, 0x56, 0x54, 0x5a,
    0x58, 0x5e, 0x5c, 0x74, 0x72, 0x70, 0x76, 0x7c, 0x7e, 0x7a, 0x78, 0x68, 0x6a};
constexpr Word external_first = 0x10, dma_first = 0x18, dma_last = 0x22;
constexpr Word dax_first = 0x28, esai_first = 0x30, shi_first = 0x40;
constexpr Word timer_first = 0x54, efcop_first = 0x68, esai1_first = 0x70;
constexpr unsigned rank_bits = 6;

struct Source {
  std::uint8_t rank, shift;
  bool core;
};

constexpr unsigned priority_shift(const Word vector) noexcept {
  if (vector < dma_first) { return ((vector - external_first) / 2) * 3; }
  if (vector <= dma_last) { return 12 + (vector - dma_first); }
  if (vector >= esai1_first) { return 10; }
  if (vector >= efcop_first) { return 16; }
  if (vector >= timer_first) { return 8; }
  if (vector >= shi_first) { return 2; }
  return vector < esai_first && vector >= dax_first ? 6 : 0;
}

constexpr auto sources = [] {
  constexpr std::size_t vector_slots = 0x80;
  static_assert(order.size() < (1U << rank_bits));
  std::array<Source, vector_slots> result{};
  for (std::size_t index = 0; index < order.size(); ++index) {
    const auto vector = order[index];
    result[vector] = {.rank = static_cast<std::uint8_t>(order.size() - index),
                      .shift = static_cast<std::uint8_t>(priority_shift(vector)),
                      .core = vector <= dma_last};
  }
  return result;
}();
}

std::optional<Interrupts::Request> Interrupts::select(const std::span<const Word> vectors,
                                                      const Word status) const noexcept {
  const auto mask = interrupt_mask(status);
  unsigned best = 0;
  Word winner = 0;
  for (const auto vector : vectors) {
    if (vector == 0 || vector >= sources.size()) { continue; }
    const auto source = sources[vector];
    if (source.rank == 0) { continue; }
    const auto configured = ((source.core ? core_ : peripheral_) >> source.shift) & 3U;
    const auto key = (configured << rank_bits) | source.rank;
    if (configured > mask && key > best) {
      best = key;
      winner = vector;
    }
  }
  if (best == 0) { return {}; }
  return Request{.vector = winner, .level = (best >> rank_bits) - 1U};
}
}
