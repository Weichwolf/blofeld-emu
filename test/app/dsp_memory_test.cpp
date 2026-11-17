// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/boot_rom.hpp"
#include "dsp/memory.hpp"
#include "dsp_memory.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main() try {
  using namespace blofeld;
  using dsp56300::Space;
  using dsp56300::Word;
  constexpr Word value = 0x123456;
  constexpr Word low_address = 17;
  constexpr Word reserved_y = 0x16000;
  constexpr Word discarded_y = 0x1ffff;
  constexpr Word p_y_first = 0x3000;
  constexpr Word high_first = 0xc000;
  std::array<Word, vi::dsp::BootRom::document_words> words{};
  words[0] = value;
  words[vi::dsp::BootRom::main_words] = value + 1;
  const app::DspMemory backing(vi::dsp::BootRom::document(words));
  vi::dsp::Memory memory({}, backing.storage());
  Word result = 0;
  require(!memory.read(Space::program, 4, result),
          "Low P code is unknown before its actual upload");
  require(memory.write({.space = Space::program, .address = 4, .value = value}) &&
              memory.read(Space::program, 4, result) && result == value,
          "Original P stores initialize the same physical code inventory");
  constexpr std::array modes{Word{0}, Word{0x80}, Word{0x200080}, Word{0x400080}, Word{0x600080}};
  constexpr std::array y_sizes{
      Word{0xa000}, Word{0x2000}, Word{0x4000}, Word{0x8000}, Word{0xa000}};
  for (std::size_t index = 0; index < modes.size(); ++index) {
    memory.configure(modes[index]);
    static_cast<void>(memory.loan()); // Cold validation of every complete public loan.
    const auto &views = backing.storage().mappings[index].views;
    const auto *const p = static_cast<const volatile Word *>(views[0].read.data.data());
    auto *const x = static_cast<volatile Word *>(views[1].write.data.data());
    auto *const y = static_cast<volatile Word *>(views[2].write.data.data());
    const auto *const py = static_cast<const volatile Word *>(views[2].read.data.data());
    require(p[vi::dsp::BootRom::main_first] == value &&
                p[vi::dsp::BootRom::eeprom_first] == value + 1 && p[4] == value,
            "Every map preserves the original ROM fragments and uploaded low P contents");
    x[low_address] = value + static_cast<Word>(index);
    require(memory.read(Space::x, low_address, result) && result == value + index,
            "Native stores and DMA/consumer reads share the same physical RAM");
    y[reserved_y] = value;
    y[discarded_y] = value;
    require(py[reserved_y] == 0 && py[dsp56300::word_mask] == 0,
            "Reserved constant reads are isolated from discarded stores, including mixed pages");
    if (index != 0) {
      auto *const write_p = static_cast<volatile Word *>(views[0].write.data.data());
      write_p[0x1000] = value + 10;
      if (y_sizes[index] != y_sizes.front()) { write_p[p_y_first] = value + 11; }
      memory.configure(0);
      require(memory.read(Space::x, 0x10000, result) && result == value + 10,
              "Switched P aliases the retained high-X tail");
      if (y_sizes[index] != y_sizes.front()) {
        require(memory.read(Space::y, high_first + y_sizes[index], result) && result == value + 11,
                "Every switched P/Y partition aliases its retained physical tail");
      }
    }
  }
  require(!memory.read(Space::x, vi::dsp::Memory::bank_words[1], result),
          "Reserved write ranges never become readable RAM");
  std::cout << "DSP shared physical banks, five OMR loans, ROM and scratch isolation passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
