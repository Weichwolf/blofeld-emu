// SPDX-License-Identifier: GPL-3.0-only
#include "storage/firmware.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<std::uint8_t> fixture(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  const auto size = input.tellg();
  require(size > 0, "Readable independent unit input");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  input.seekg(0);
  require(static_cast<bool>(input.read(reinterpret_cast<char *>(bytes.data()),
                                       static_cast<std::streamsize>(bytes.size()))),
          "Complete unit input");
  return bytes;
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  const std::filesystem::path root(argv[2]);
  const auto input = fixture(root / "update.mid");
  const auto update = blofeld::vi::FirmwareUpdate::from_midi(input);
  std::array<std::uint8_t, blofeld::vi::FirmwareUpdate::flash_bytes> expected{};
  constexpr std::size_t application = 512;
  constexpr std::size_t checksum = 0x7f10;
  constexpr std::uint8_t boot_pattern = 0xa5;
  constexpr std::uint8_t high = 0x12;
  constexpr std::uint8_t low = 0x34;
  expected[0] = boot_pattern;
  expected[application] = high;
  expected[application + 1] = low;
  expected[checksum] = high;
  expected[checksum + 1] = low;
  require(update.mcu_flash == expected,
          "All decoded flash bytes match independently authored data");
  constexpr std::size_t main_bytes = 32826;
  constexpr std::size_t nand_bytes = 51;
  require(update.blocks[4].size() == main_bytes && update.nand_program.size() == nand_bytes,
          "Declared main block and complete NAND words have distinct bounds");
  constexpr unsigned padding = 0x5a;
  require(update.nand_program.front() == 0 && update.nand_program.back() == padding,
          "Preserve actual transport padding rather than inventing erased bytes");
  for (const auto *name : {"bad-packet.mid", "bad-app.mid"}) {
    bool rejected = false;
    try {
      static_cast<void>(blofeld::vi::FirmwareUpdate::from_midi(fixture(root / name)));
    } catch (const std::runtime_error &error) {
      const std::string_view message(error.what());
      rejected = message.contains("XOR") || message.contains("boot checksum");
    }
    require(rejected, "Reject broken transport XOR or MCU application checksum");
  }
  bool rejected = false;
  try {
    static_cast<void>(blofeld::vi::FirmwareUpdate::from_midi({}));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Reject incomplete MIDI container");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
