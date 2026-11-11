// SPDX-License-Identifier: GPL-3.0-only
#include "boot_rom.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace {
using blofeld::app::BootFormat;
using blofeld::app::load_boot_rom;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(const std::filesystem::path &path, const std::span<const std::uint8_t> bytes) {
  std::ofstream output(path, std::ios::binary);
  if (!bytes.empty()) {
    output.write(reinterpret_cast<const char *>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
  }
  require(output.good(), "Write owned boot input");
}

void valid(const std::filesystem::path &path, const BootFormat format, const bool eeprom = false) {
  constexpr std::array<std::uint8_t, 6> head{0x12, 0x34, 0x56, 0xab, 0xcd, 0xef};
  constexpr std::array<std::uint8_t, 3> tail{0x80, 0, 1};
  constexpr std::size_t full_words = 192;
  constexpr std::size_t prefix_words = 55;
  constexpr std::size_t document_words = 273;
  auto words = format == BootFormat::rom ? full_words : prefix_words;
  if (eeprom) { words = document_words; }
  std::vector<std::uint8_t> bytes(words * 3);
  std::ranges::copy(head, bytes.begin());
  std::ranges::copy(tail, bytes.end() - static_cast<std::ptrdiff_t>(tail.size()));
  write(path, bytes);
  const auto decoded = load_boot_rom(path, format);
  constexpr std::uint32_t first = 0x123456;
  constexpr std::uint32_t second = 0xabcdef;
  constexpr std::uint32_t last = 0x800001;
  require(decoded.size() == words && decoded[0] == first && decoded[1] == second &&
              decoded.back() == last &&
              std::ranges::all_of(std::span(decoded).subspan(2, words - 3),
                                  [](const auto value) { return value == 0; }),
          "Boot input preserves exact 24-bit words and never pads unknown trailing ROM");
}

void invalid_lengths(const std::filesystem::path &path, const BootFormat format) {
  constexpr std::array<std::size_t, 11> lengths{0, 1, 3, 162, 168, 575, 579, 816, 818, 820, 822};
  for (const auto length : lengths) {
    write(path, std::vector<std::uint8_t>(length));
    bool rejected = false;
    try {
      static_cast<void>(load_boot_rom(path, format));
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Boot input rejects empty, truncated, oversized and mismatched formats");
  }
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned boot unit directory");
  const auto file = root / "boot.bin";
  valid(file, BootFormat::rom);
  bool full_rejected_as_prefix = false;
  try {
    static_cast<void>(load_boot_rom(file, BootFormat::spi_prefix));
  } catch (const std::runtime_error &) { full_rejected_as_prefix = true; }
  require(full_rejected_as_prefix, "A prefix field cannot silently accept a full-ROM image");
  invalid_lengths(file, BootFormat::spi_prefix);
  valid(file, BootFormat::spi_prefix);
  bool rejected = false;
  try {
    static_cast<void>(load_boot_rom(file, BootFormat::rom));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "A raw full-ROM field cannot silently accept the partial SPI prefix");
  invalid_lengths(file, BootFormat::rom);
  valid(file, BootFormat::rom, true);
  rejected = false;
  try {
    static_cast<void>(load_boot_rom(file, BootFormat::spi_prefix));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "An explicit prefix cannot accept a compact main/EEPROM ROM package");
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
