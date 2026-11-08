// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"
#include "storage/media_image.hpp"

#include <algorithm>
#include <cstdint>
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
  using blofeld::vi::MediaImage;
  const auto root = std::filesystem::path(argv[2]);
  auto bytes = blofeld::host::read_file(root / "media.bfnand", MediaImage::max_bytes);
  const MediaImage image(bytes);
  std::uint8_t value = 0;
  constexpr MediaImage::Read end_of_block{.page = 31, .column = 527};
  constexpr MediaImage::Read unknown_bits{.page = 32, .column = 17};
  constexpr MediaImage::Read last_byte{.page = 131071, .column = 527};
  constexpr std::uint8_t expected_end = 46;
  constexpr std::uint8_t expected_last = 14;
  require(image.read({}, value) && value == 0 && image.read(end_of_block, value) &&
              value == expected_end && image.read(last_byte, value) && value == expected_last,
          "Read independently generated page data at exact geometry boundaries");
  require(!image.read(unknown_bits, value) && !image.read({.page = 1}, value) &&
              !image.read({.page = MediaImage::pages}, value) &&
              !image.read({.column = MediaImage::page_bytes}, value),
          "Unknown bits, missing pages and invalid coordinates never become erased bytes");
  require(std::ranges::equal(image.encoded(), bytes),
          "Retain complete original encoded metadata and bytes");
  const MediaImage empty;
  require(!empty.read({}, value), "An absent page remains unknown");
  bytes.back() ^= 1;
  bool rejected = false;
  try {
    const MediaImage bad(bytes);
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Reject CRC corruption");
  rejected = false;
  try {
    const MediaImage bad(
        blofeld::host::read_file(root / "bad-media.bfnand", MediaImage::max_bytes));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Reject noncanonical unknown bits even with a correct CRC");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
