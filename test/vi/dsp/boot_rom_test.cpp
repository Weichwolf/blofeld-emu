// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/boot_rom.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
using blofeld::vi::dsp::BootRom;
// Independent document/input expectations, not values copied from the reader.
constexpr std::uint32_t main_first = 0xff0000, main_last = 0xff00bf;
constexpr std::uint32_t eeprom_first = 0xffff00, eeprom_last = 0xffff50;
constexpr std::uint32_t address_end = 0x1000000, eeprom_value = 0xabcdef;
constexpr std::uint32_t prefix_last = 0xff0036;
constexpr std::size_t main_words = 192, eeprom_words = 81, document_words = 273;
constexpr std::size_t prefix_words = 55;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void mapped_regions() {
  std::array<std::uint32_t, main_words> main{};
  std::array<std::uint32_t, eeprom_words> eeprom{};
  std::uint32_t value = 0;
  for (auto &word : main) { word = value++; }
  value = eeprom_value;
  for (auto &word : eeprom) { word = value++; }
  const BootRom rom{.main = main, .eeprom = eeprom};
  require(rom.valid(), "Disjoint known ROM regions have valid bounds");
  for (std::uint32_t address = main_first; address < address_end; ++address) {
    std::optional<std::uint32_t> expected;
    if (address <= main_last) { expected = address - main_first; }
    if (address >= eeprom_first && address <= eeprom_last) {
      expected = address - eeprom_first + eeprom_value;
    }
    require(
        rom.read(address) == expected,
        "Exactly the two documented ranges are readable; every intervening/tail word is unknown");
  }
  require(!rom.read(main_first - 1) && !rom.read(address_end) &&
              !rom.read(std::numeric_limits<std::uint32_t>::max()),
          "ROM reads reject lower and out-of-width addresses without arithmetic wraparound");
  const BootRom prefix{.main = std::span(main).first(prefix_words), .eeprom = {}};
  require(prefix.valid() && prefix.read(prefix_last) == prefix_words - 1 &&
              !prefix.read(prefix_last + 1) && !prefix.read(eeprom_first),
          "A supplied prefix exposes neither the main tail nor an invented EEPROM region");
}

void document_views() {
  std::array<std::uint32_t, document_words> words{};
  const auto full = BootRom::document(words);
  require(full.valid() && full.main.size() == main_words && full.eeprom.size() == eeprom_words &&
              full.main.data() == words.data() && full.eeprom.data() == words.data() + main_words,
          "The compact document package borrows both actual ranges without padding or copies");
  for (const std::size_t count : {prefix_words, main_words}) {
    const auto part = BootRom::document(std::span(words).first(count));
    require(part.valid() && part.main.size() == count && part.main.data() == words.data() &&
                part.eeprom.empty(),
            "Existing explicit prefix/main packages retain exactly their supplied words");
  }
  require(!BootRom{}.valid(), "The reset entry requires supplied main ROM");
  const std::vector<std::uint32_t> crossing(eeprom_first - main_first + 1);
  const std::array<std::uint32_t, 1> one{};
  require(!BootRom{.main = crossing, .eeprom = one}.valid(), "Known regions cannot overlap");
  const std::vector<std::uint32_t> oversized(address_end - eeprom_first + 1);
  require(!BootRom{.main = one, .eeprom = oversized}.valid(),
          "The EEPROM region cannot cross the 24-bit program-space boundary");
}
}

int main() try {
  mapped_regions();
  document_views();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
