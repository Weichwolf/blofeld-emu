// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"
#include "storage/media_image.hpp"

#include <algorithm>
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

void mutations(const std::filesystem::path &root) {
  using blofeld::vi::MediaImage;
  MediaImage image(blofeld::host::read_file(root / "media-writable.bfnand", MediaImage::max_bytes));
  constexpr std::uint8_t erased = 0xff;
  constexpr unsigned unknown_column = 17;
  constexpr unsigned last_column = MediaImage::page_bytes - 1;
  constexpr unsigned last_page = MediaImage::pages - 1;
  constexpr unsigned final_block = (MediaImage::pages / MediaImage::block_pages) - 1;
  require(image.erase(0) == true, "Erase the entire selected block, materializing missing pages");
  std::uint8_t value = 0;
  for (unsigned page = 0; page < MediaImage::block_pages; ++page) {
    for (unsigned column = 0; column < MediaImage::page_bytes; ++column) {
      require(image.read({.page = page, .column = column}, value) && value == erased,
              "Every main/spare byte in a successfully erased block is known FF");
    }
  }
  require(image.read({.page = MediaImage::block_pages}, value) &&
              value == MediaImage::block_pages &&
              !image.read({.page = MediaImage::block_pages, .column = unknown_column}, value),
          "Erasure leaves the next block's known and unknown data unchanged");
  auto encoded = image.encoded();
  const MediaImage restored(std::vector<std::uint8_t>(encoded.begin(), encoded.end()));
  require(restored.read({.page = 1, .column = last_column}, value) && value == erased,
          "Materialized pages round trip through validated, ordered records and refreshed CRC");
  image.interrupt_erase(0);
  require(!image.read({}, value) &&
              !image.read({.page = MediaImage::block_pages - 1, .column = last_column}, value),
          "Interrupted erase makes the affected data unknown, preserving recovery uncertainty");
  encoded = image.encoded();
  const MediaImage interrupted(std::vector<std::uint8_t>(encoded.begin(), encoded.end()));
  require(!interrupted.read({}, value), "Interrupted data stays unknown after image reload");
  require(image.erase(0) == true && image.erase(1) == true,
          "A known valid block can be erased again; erase clears program-only protection");
  encoded = image.encoded();
  const auto before = std::vector<std::uint8_t>(encoded.begin(), encoded.end());
  require(image.erase(2) == false && image.erase(3) == false && !image.erase(4).has_value() &&
              !image.erase(MediaImage::pages / MediaImage::block_pages).has_value() &&
              std::ranges::equal(image.encoded(), before),
          "Protected, invalid, unknown and out-of-range blocks never produce successful writes");
  require(image.erase(final_block) == true &&
              image.read({.page = last_page, .column = last_column}, value) && value == erased,
          "The final block materializes correctly without wrapping the image");
  encoded = image.encoded();
  const MediaImage final_image(std::vector<std::uint8_t>(encoded.begin(), encoded.end()));
  require(final_image.read({.page = MediaImage::pages - MediaImage::block_pages}, value) &&
              value == erased,
          "The final block remains canonical after serialization");
}

}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  using blofeld::vi::MediaImage;
  constexpr unsigned last_column = MediaImage::page_bytes - 1;
  constexpr unsigned unknown_column = 17;
  constexpr unsigned last_page = MediaImage::pages - 1;
  const auto root = std::filesystem::path(argv[2]);
  mutations(root);
  auto bytes = blofeld::host::read_file(root / "media.bfnand", MediaImage::max_bytes);
  MediaImage image(bytes);
  std::uint8_t value = 0;
  constexpr MediaImage::Read end_of_block{.page = MediaImage::block_pages - 1,
                                          .column = last_column};
  constexpr MediaImage::Read unknown_bits{.page = MediaImage::block_pages,
                                          .column = unknown_column};
  constexpr MediaImage::Read last_byte{.page = last_page, .column = last_column};
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
