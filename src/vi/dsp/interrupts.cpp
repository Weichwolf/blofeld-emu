// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/interrupts.hpp"

#include <algorithm>
#include <array>
#include <dsp56300/memory.hpp>
#include <optional>
#include <span>
#include <vector>

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
}

unsigned Interrupts::priority(const Word vector) const noexcept {
  unsigned shift = 0;
  if (vector < dma_first) {
    shift = ((vector - external_first) / 2) * 3;
    return (core_ >> shift) & 3;
  }
  if (vector <= dma_last) {
    shift = 12 + (vector - dma_first);
    return (core_ >> shift) & 3;
  }
  if (vector >= esai1_first) {
    shift = 10;
  } else if (vector >= efcop_first) {
    shift = 16;
  } else if (vector >= timer_first) {
    shift = 8;
  } else if (vector >= shi_first) {
    shift = 2;
  } else if (vector >= esai_first) {
    shift = 0;
  } else if (vector >= dax_first) {
    shift = 6;
  }
  return (peripheral_ >> shift) & 3;
}

std::vector<Word> Interrupts::entries(const Word vba) const {
  constexpr Word base_mask = 0xffff00;
  std::vector<Word> result;
  result.reserve(order.size());
  for (const auto vector : order) {
    if (priority(vector) != 0) { result.push_back((vba & base_mask) | vector); }
  }
  return result;
}

std::optional<Interrupts::Request> Interrupts::select(const std::span<const Word> vectors,
                                                      const Word status) const noexcept {
  const auto mask = (status >> 8U) & 3;
  std::optional<Request> result;
  const auto *previous = order.end();
  for (const auto vector : vectors) {
    if (vector == 0) { continue; }
    const auto *const rank = std::ranges::find(order, vector);
    if (rank == order.end()) { continue; }
    const auto configured = priority(vector);
    if (configured == 0 || configured - 1 < mask) { continue; }
    const auto level = configured - 1;
    if (!result || level > result->level || (level == result->level && rank < previous)) {
      result = Request{.vector = vector, .level = level};
      previous = rank;
    }
  }
  return result;
}
}
