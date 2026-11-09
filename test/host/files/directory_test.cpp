// SPDX-License-Identifier: GPL-3.0-only
#include "files/directory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<std::uint8_t> read(const std::filesystem::path &file) {
  std::ifstream stream(file, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create an owned test directory");
  {
    const blofeld::host::Directory directory(root);
    constexpr std::array<std::uint8_t, 3> first{1, 2, 3};
    directory.replace("frame", first);
    require(read(root / "frame") == std::vector<std::uint8_t>{1, 2, 3}, "Complete initial file");
    std::ifstream old(root / "frame", std::ios::binary);
    constexpr std::array<std::uint8_t, 2> next{4, 5};
    directory.replace("frame", next);
    require(old.get() == 1 && read(root / "frame") == std::vector<std::uint8_t>{4, 5},
            "Existing readers keep the old inode; new readers see complete replacement");
    bool rejected = false;
    try {
      directory.replace("../outside", first);
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Publication cannot leave its directory");
    rejected = false;
    try {
      const blofeld::host::Directory competitor(root);
    } catch (const std::system_error &) { rejected = true; }
    require(rejected, "Only one writer owns a directory");
    directory.remove("frame");
    directory.remove("frame");
    require(!std::filesystem::exists(root / "frame"), "Removal is confined and idempotent");
    rejected = false;
    try {
      directory.remove("../outside");
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Removal cannot leave its directory");
    std::filesystem::rename(root, root.string() + ".moved");
    std::filesystem::create_directory(root);
    directory.replace("frame", first);
    require(!std::filesystem::exists(root / "frame") &&
                read(root.string() + ".moved/frame") == std::vector<std::uint8_t>{1, 2, 3},
            "Path replacement does not redirect publication");
  }
  std::filesystem::remove_all(root);
  std::filesystem::remove_all(root.string() + ".moved");
} catch (const std::exception &error) {

  std::cerr << error.what() << "\n";
  return 1;
}
