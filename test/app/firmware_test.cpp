// SPDX-License-Identifier: GPL-3.0-only
#include "firmware.hpp"
#include "storage/firmware.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  const std::filesystem::path root(argv[2]);
  const auto midi = blofeld::app::load_firmware(root / "update.mid");
  const auto zip = blofeld::app::load_firmware(root / "update.zip");
  constexpr std::size_t expected_nand_bytes = 51;
  require(zip.nand_program.size() == expected_nand_bytes && midi.mcu_flash == zip.mcu_flash &&
              midi.blocks == zip.blocks && midi.nand_program == zip.nand_program,
          "Archive/file selection preserves the complete independently authored update");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
