// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_memory.hpp"

#include "dsp/boot_rom.hpp"
#include "dsp/memory.hpp"
#include "memory/address_space.hpp"
#include "process/configuration.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <memory>
#include <span>
#include <stdexcept>
#include <unistd.h>

namespace blofeld::app {
namespace {
using Word = dsp56300::Word;
constexpr std::size_t high_first = 0xc000, p_y_first = 0x3000, reserved_y = 0x16000;
constexpr auto ram_words = vi::dsp::Memory::physical_words;
constexpr auto space_bytes = dsp56300::address_words * sizeof(Word);
constexpr std::array<std::size_t, 5> offsets{0, 0x1000, 0x4000, 0x6000, 0xc000};
constexpr std::array<std::size_t, 5> y_sizes{0xa000, 0x2000, 0x4000, 0x8000, 0xa000};

std::size_t page_size() {
  const auto page = sysconf(_SC_PAGESIZE);
  if (page <= 0 || static_cast<std::size_t>(page) % sizeof(Word) != 0) {
    throw std::runtime_error("DSP mappings require a word-aligned host page size");
  }
  return static_cast<std::size_t>(page);
}

std::span<Word> words(const std::span<std::byte> data) {
  return {reinterpret_cast<Word *>(data.data()), data.size() / sizeof(Word)};
}
}

dsp56300::CompileOptions::MemoryMode dsp_memory_mode() {
  const auto setting = host::inherited_setting("BLOFELD_DSP_MEMORY");
  if (setting.empty() || setting == "direct") {
    return dsp56300::CompileOptions::MemoryMode::direct;
  }
  if (setting == "exact") { return dsp56300::CompileOptions::MemoryMode::exact; }
  throw std::invalid_argument("BLOFELD_DSP_MEMORY must be direct or exact");
}

DspMemory::DspMemory(const vi::dsp::BootRom boot)
    : page_(page_size()), backing_((ram_words * sizeof(Word)) + (page_ * 2)) {
  const auto tail = (vi::dsp::BootRom::eeprom_first * sizeof(Word)) % page_;
  if (!boot.valid() || boot.main.size_bytes() > page_ || tail + boot.eeprom.size_bytes() > page_) {
    throw std::invalid_argument("DSP boot fragments do not fit the declared host pages");
  }
  auto physical = words(backing_.bytes());
  std::ranges::copy(boot.main, physical.subspan(ram_words).begin());
  std::ranges::copy(boot.eeprom,
                    physical.subspan(ram_words + ((page_ + tail) / sizeof(Word))).begin());
  const bool has_eeprom = !boot.eeprom.empty();
  for (std::size_t index = 0; index < mappings_.size(); ++index) {
    mapping(index, has_eeprom);
    const auto p = mappings_[index].views[0].read.data;
    program_[index] = {{{.address = vi::dsp::BootRom::main_first,
                         .words = p.subspan(vi::dsp::BootRom::main_first, boot.main.size()),
                         .known = {}},
                        {.address = vi::dsp::BootRom::eeprom_first,
                         .words = p.subspan(vi::dsp::BootRom::eeprom_first, boot.eeprom.size()),
                         .known = {}}}};
    mappings_[index].program = std::span(program_[index]).first(has_eeprom ? 2U : 1U);
  }
}

void DspMemory::mapping(const std::size_t index, const bool has_eeprom) {
  auto &mapping = mappings_[index];
  for (std::size_t bank = 0; bank < mapping.views.size(); ++bank) {
    for (const bool writing : {false, true}) {
      auto &space = spaces_[(index * 6) + (bank * 2) + static_cast<std::size_t>(writing)];
      space = std::make_unique<host::memory::AddressSpace>(space_bytes);
      if (writing) {
        const auto first = bank == 0 ? vi::dsp::BootRom::main_first * sizeof(Word) : 0;
        const auto last = bank == 0 ? space_bytes : vi::dsp::BootRom::main_first * sizeof(Word);
        space->map({.address = first, .size = last - first, .writable = true});
      }
      (writing ? mapping.views[bank].write : mapping.views[bank].read).data = words(space->bytes());
    }
  }
  const auto map = [&](const std::size_t bank,
                       const host::memory::AddressSpace::Region region,
                       const host::memory::Backing *backing) {
    for (const bool writing : {false, true}) {
      auto copy = region;
      copy.writable = writing;
      spaces_[(index * 6) + (bank * 2) + static_cast<std::size_t>(writing)]->map(copy, backing);
    }
  };
  const auto ram = [&](const std::size_t bank,
                       const std::size_t address,
                       const std::size_t offset,
                       const std::size_t count) {
    if (count != 0) {
      map(bank,
          {.address = address * sizeof(Word),
           .offset = offset * sizeof(Word),
           .size = count * sizeof(Word)},
          &backing_);
    }
  };
  ram(0, 0, offsets[0], 0x1000);
  ram(1, 0, offsets[1], vi::dsp::Memory::bank_words[1]);
  ram(2, 0, offsets[2], 0x2000);
  ram(1, high_first, offsets[3], index == 0 ? vi::dsp::Memory::bank_words[3] : 0x4000);
  ram(2, high_first, offsets[4], y_sizes[index]);
  if (index != 0) {
    ram(0, 0x1000, offsets[3] + 0x4000, 0x2000);
    ram(0, p_y_first, offsets[4] + y_sizes[index], y_sizes.front() - y_sizes[index]);
  }
  auto &p = *spaces_[index * 6];
  p.map({.address = vi::dsp::BootRom::main_first * sizeof(Word),
         .offset = ram_words * sizeof(Word),
         .size = page_},
        &backing_);
  if (has_eeprom) {
    p.map({.address = (vi::dsp::BootRom::eeprom_first * sizeof(Word)) & ~(page_ - 1),
           .offset = (ram_words * sizeof(Word)) + page_,
           .size = page_},
          &backing_);
  }
  auto &y = *spaces_[(index * 6) + 4];
  for (const auto address : {reserved_y, std::size_t{dsp56300::word_mask}}) {
    y.map({.address = (address * sizeof(Word)) & ~(page_ - 1), .size = page_});
  }
}

vi::dsp::Memory::Storage DspMemory::storage() const noexcept {
  return {.physical = words(backing_.bytes()).first(ram_words), .mappings = mappings_};
}
}
