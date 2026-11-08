// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned unit directory");
  {
    std::ofstream input(root / "input", std::ios::binary);
    input << "abc";
  }
  require(blofeld::host::read_file(root / "input", 3) == std::vector<std::uint8_t>{'a', 'b', 'c'},
          "Read exact binary bytes");
  bool rejected = false;
  try {
    static_cast<void>(blofeld::host::read_file(root / "input", 2));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Reject oversized input");
  std::filesystem::create_symlink("input", root / "link");
  rejected = false;
  try {
    static_cast<void>(blofeld::host::read_file(root / "link", 3));
  } catch (const std::system_error &) { rejected = true; }
  require(rejected, "Do not follow input symlinks");
  constexpr unsigned private_mode = 0600;
  require(::mkfifo((root / "fifo").c_str(), private_mode) == 0, "Create FIFO");
  rejected = false;
  try {
    static_cast<void>(blofeld::host::read_file(root / "fifo", 3));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Non-regular inputs fail without blocking");
  { const std::ofstream empty(root / "empty"); }
  require(blofeld::host::read_file(root / "empty", 0).empty(), "Empty regular file is valid");
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
