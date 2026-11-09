// SPDX-License-Identifier: GPL-3.0-only
#include "boot_rom.hpp"

#include "files/read.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace blofeld::app {
std::vector<std::uint32_t> load_boot_rom(const std::filesystem::path &path,
                                         const BootFormat format) {
  constexpr std::size_t word_bytes = 3;
  constexpr std::size_t rom_words = 192;
  constexpr std::size_t prefix_words = 55;
  std::size_t words = 0;
  switch (format) {
    case BootFormat::rom: words = rom_words; break;
    case BootFormat::spi_prefix: words = prefix_words; break;
    default: throw std::invalid_argument("Unknown DSP boot input format");
  }
  const auto input = host::read_file(path, words * word_bytes);
  if (input.size() != words * word_bytes) {
    throw std::runtime_error("DSP boot input requires exactly " + std::to_string(words) +
                             " big-endian 24-bit words");
  }
  std::vector<std::uint32_t> result;
  result.reserve(words);
  for (std::size_t at = 0; at < input.size(); at += word_bytes) {
    result.push_back((std::uint32_t{input[at]} << 16U) | (std::uint32_t{input[at + 1]} << 8U) |
                     input[at + 2]);
  }
  return result;
}
}
