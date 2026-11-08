// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/memory.hpp"

#include <dsp56300/memory.hpp>
#include <span>

namespace blofeld::vi::dsp {
Memory::Memory()
    : x_window_{{{.address = 0, .words = x_.words, .known = x_.known, .writable = true}}},
      y_window_{{{.address = 0, .words = y_.words, .known = y_.known, .writable = true}}} {}

bool Memory::read(const dsp56300::Space space,
                  const dsp56300::Word address,
                  dsp56300::Word &value) const noexcept {
  const auto load = [&](const auto &block) {
    if (address >= block.words.size() || block.known[address] == 0) { return false; }
    value = block.words[address];
    return true;
  };
  switch (space) {
    case dsp56300::Space::program: return load(program_);
    case dsp56300::Space::x: return load(x_);
    case dsp56300::Space::y: return load(y_);
    default: return false;
  }
}

bool Memory::write(const Write access) noexcept {
  if (access.value > dsp56300::word_mask) { return false; }
  const auto store = [&](auto &block) {
    if (access.address >= block.words.size()) { return false; }
    block.words[access.address] = access.value;
    block.known[access.address] = 1;
    return true;
  };
  switch (access.space) {
    case dsp56300::Space::program: return store(program_);
    case dsp56300::Space::x: return store(x_);
    case dsp56300::Space::y: return store(y_);
    default: return false;
  }
}

std::span<const dsp56300::MemoryWindow> Memory::windows(const dsp56300::Space space) noexcept {
  switch (space) {
    case dsp56300::Space::x: return x_window_;
    case dsp56300::Space::y: return y_window_;
    case dsp56300::Space::program:
    default: return {};
  }
}
}
