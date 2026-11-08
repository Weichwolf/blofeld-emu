// SPDX-License-Identifier: GPL-3.0-only
#include "archive/zip.hpp"
#include "files/read.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply unit artifact and fixture directories");
  const std::filesystem::path root(argv[2]);
  constexpr std::size_t limit = 4096;
  const std::vector<std::uint8_t> expected{'a', 'b', 'c', 0, 'd', 'e', 'f'};
  for (const auto *name : {"stored.zip", "deflated.zip"}) {
    const auto bytes = blofeld::host::read_file(root / name, limit);
    require(blofeld::host::zip_member(bytes, ".mid", limit) == expected,
            "Independent stored/deflated contents and case-insensitive suffix");
    bool rejected = false;
    try {
      static_cast<void>(blofeld::host::zip_member(bytes, ".mid", 3));
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Declared unpacked size is bounded");
  }
  const auto empty = blofeld::host::read_file(root / "empty.zip", limit);
  require(blofeld::host::zip_member(empty, ".mid", 0).empty(), "Empty deflated member");
  for (const auto *name : {"ambiguous.zip", "damaged.zip"}) {
    const auto bytes = blofeld::host::read_file(root / name, limit);
    bool rejected = false;
    try {
      static_cast<void>(blofeld::host::zip_member(bytes, ".mid", limit));
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Ambiguous selection and actual CRC corruption fail");
  }
  bool rejected = false;
  try {
    static_cast<void>(blofeld::host::zip_member({}, ".mid", limit));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Truncated directory fails");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
