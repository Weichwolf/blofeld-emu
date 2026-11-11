// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi::dsp {
struct BootRom {
  static constexpr std::uint32_t main_first = 0xff0000, eeprom_first = 0xffff00;
  static constexpr std::size_t main_words = 192, eeprom_words = 81;
  static constexpr std::size_t document_words = main_words + eeprom_words;
  std::span<const std::uint32_t> main, eeprom;

  [[nodiscard]] static BootRom document(const std::span<const std::uint32_t> words) noexcept {
    if (words.size() == document_words) {
      return {.main = words.first(main_words), .eeprom = words.subspan(main_words)};
    }
    return {.main = words, .eeprom = {}};
  }

  [[nodiscard]] bool valid() const noexcept {
    constexpr std::uint32_t end = 0x1000000;
    const auto limit = eeprom.empty() ? end : eeprom_first;
    return !main.empty() && main.size() <= limit - main_first &&
           eeprom.size() <= end - eeprom_first;
  }

  [[nodiscard]] std::optional<std::uint32_t> read(const std::uint32_t address) const noexcept {
    if (address >= main_first && address - main_first < main.size()) {
      return main[address - main_first];
    }
    if (address >= eeprom_first && address - eeprom_first < eeprom.size()) {
      return eeprom[address - eeprom_first];
    }
    return {};
  }
};
}
