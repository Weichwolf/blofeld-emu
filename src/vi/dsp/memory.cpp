// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/memory.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <optional>
#include <span>
#include <stdexcept>

namespace blofeld::vi::dsp {
namespace {
constexpr dsp56300::Word rom_first = 0x4000, high_first = 0xc000;
constexpr dsp56300::Word boot_first = 0xff0000;
constexpr dsp56300::Word memory_switch = 0x80;
constexpr dsp56300::Word mapping_mask = memory_switch | (3U << 21);
constexpr std::size_t switched_x_words = 0x4000;
constexpr dsp56300::Word y_program_first = 0x3000;
constexpr dsp56300::Word reserved_y_last = 0xffffff;
constexpr dsp56300::Word reserved_y_boundary = 0x16000;
}

Memory::Memory(const Rom rom) : rom_(rom) {
  for (const auto words : {rom.x, rom.y}) {
    if (words.size() > high_first - rom_first ||
        std::ranges::any_of(words, [](const auto word) { return word > dsp56300::word_mask; })) {
      throw std::invalid_argument("DSP data ROM has invalid bounds or word width");
    }
  }
  program_windows_[0] = {
      .address = 0, .words = program_.words, .known = program_.known, .writable = true};
  configure(0);
}

void Memory::configure(const dsp56300::Word omr) noexcept {
  const bool switched = (omr & memory_switch) != 0;
  const auto mapping = switched ? omr & mapping_mask : 0U;
  if (mapping == mapping_) { return; }
  mapping_ = mapping;
  constexpr std::array<std::size_t, 4> y_sizes{0x2000, 0x4000, 0x8000, 0xa000};
  const auto x_size = switched ? switched_x_words : high_x_.words.size();
  const auto y_size = switched ? y_sizes[(omr >> 21U) & 3] : high_y_.words.size();
  const auto bind = [](auto &pages,
                       auto &low,
                       auto &high,
                       const std::span<dsp56300::Word> rom_words,
                       const std::size_t count) {
    pages[0] = {.address = 0, .words = low.words, .known = low.known, .writable = true};
    unsigned next = 1;
    if (!rom_words.empty()) {
      pages[next++] = {.address = rom_first, .words = rom_words, .known = {}};
    }
    pages[next] = {.address = high_first,
                   .words = std::span(high.words).first(count),
                   .known = std::span(high.known).first(count),
                   .writable = true};
  };
  bind(x_windows_, x_, high_x_, rom_.x, x_size);
  bind(y_windows_, y_, high_y_, rom_.y, y_size);
  if (initialized_x_) { x_windows_[0].known = {}; }
  if (initialized_y_) { y_windows_[0].known = {}; }
  program_windows_[1] = {.address = 0x1000,
                         .words = std::span(high_x_.words).subspan(switched_x_words),
                         .known = std::span(high_x_.known).subspan(switched_x_words),
                         .writable = true};
  program_windows_[2] = {.address = y_program_first,
                         .words = std::span(high_y_.words).subspan(y_size),
                         .known = std::span(high_y_.known).subspan(y_size),
                         .writable = true};
  if (!switched) {
    program_windows_[1].words = {};
    program_windows_[1].known = {};
  }
}

std::span<const dsp56300::MemoryWindow> Memory::view(const dsp56300::Space space) const noexcept {
  switch (space) {
    case dsp56300::Space::program: return program_windows_;
    case dsp56300::Space::x: return std::span(x_windows_).first(rom_.x.empty() ? 2U : 3U);
    case dsp56300::Space::y: return std::span(y_windows_).first(rom_.y.empty() ? 2U : 3U);
    default: return {};
  }
}

std::optional<Memory::Cell> Memory::locate(const dsp56300::Space space,
                                           const dsp56300::Word address) const noexcept {
  for (const auto &page : view(space)) {
    if (address < page.address || address - page.address >= page.words.size()) { continue; }
    const auto offset = address - page.address;
    return Cell{.value = &page.words[offset],
                .known = page.known.empty() ? nullptr : &page.known[offset],
                .writable = page.writable};
  }
  return {};
}

bool Memory::read(const dsp56300::Space space,
                  const dsp56300::Word address,
                  dsp56300::Word &value) const noexcept {
  // The unchanged TASKING DSP56371 completes these reserved reads as zero.
  // Delay's staged clear leaves its first read at $16000; preserve that
  // address and its subsequent modulo update. Neither word grants RAM,
  // a status reply or a claim about physical bus data.
  if (space == dsp56300::Space::y &&
      (address == reserved_y_boundary || address == reserved_y_last)) {
    value = 0;
    return true;
  }
  const auto cell = locate(space, address);
  if (!cell || (cell->known != nullptr && *cell->known == 0)) { return false; }
  value = *cell->value;
  return true;
}

bool Memory::write(const Write access) noexcept {
  if (access.value > dsp56300::word_mask || access.address >= dsp56300::address_words) {
    return false;
  }
  if (const auto cell = locate(access.space, access.address); cell && cell->writable) {
    *cell->value = access.value;
    if (cell->known != nullptr) { *cell->known = 1; }
    return true;
  }
  switch (access.space) {
    case dsp56300::Space::x:
    case dsp56300::Space::y: return access.address < boot_first;
    case dsp56300::Space::program: return access.address >= boot_first;
    default: return false;
  }
}

std::span<const dsp56300::MemoryWindow> Memory::windows(const dsp56300::Space space) noexcept {
  return space == dsp56300::Space::program ? std::span<const dsp56300::MemoryWindow>{}
                                           : view(space);
}

bool Memory::promote_initialized() noexcept {
  bool changed = false;
  // Low X/Y RAM never aliases P, changes banks or loses initialized words.
  for (const auto space : {dsp56300::Space::x, dsp56300::Space::y}) {
    auto &initialized = space == dsp56300::Space::x ? initialized_x_ : initialized_y_;
    auto &low = space == dsp56300::Space::x ? x_windows_[0] : y_windows_[0];
    if (!initialized &&
        std::ranges::all_of(low.known, [](const auto known) { return known != 0; })) {
      initialized = true;
      low.known = {};
      changed = true;
    }
  }
  return changed;
}
}
