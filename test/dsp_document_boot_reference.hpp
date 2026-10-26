// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp_boot_reference.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test {
// Trusted private reference; the Python runner checks the pinned identity first.
inline std::vector<std::uint32_t> read_document_boot_rom(const std::filesystem::path &path) {
  constexpr std::size_t prefix_words = 192;
  std::ifstream file(path, std::ios::binary);
  std::array<char, prefix_words * 3> bytes{};
  file.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (file.gcount() != static_cast<std::streamsize>(bytes.size()) ||
      file.get() != std::char_traits<char>::eof()) {
    throw Fault("Document-derived main boot ROM must have the pinned length");
  }
  std::vector<std::uint32_t> words;
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    const std::array<std::uint8_t, 3> triplet{static_cast<std::uint8_t>(bytes[offset]),
                                              static_cast<std::uint8_t>(bytes[offset + 1]),
                                              static_cast<std::uint8_t>(bytes[offset + 2])};
    words.push_back(dsp_boot_word(triplet));
  }
  return words;
}
}
