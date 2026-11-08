// SPDX-License-Identifier: GPL-3.0-only
#include "storage/flash_image.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  const auto path = std::filesystem::path(argv[2]) / "flash.bfnvm";
  std::ifstream input(path, std::ios::binary);
  std::vector<std::uint8_t> bytes(blofeld::vi::FlashImage::file_bytes);
  require(static_cast<bool>(input.read(reinterpret_cast<char *>(bytes.data()),
                                       static_cast<std::streamsize>(bytes.size()))),
          "Read independently encoded image fixture");
  const auto image = blofeld::vi::FlashImage::decode(bytes);
  constexpr unsigned first_pattern = 0xa5;
  constexpr std::size_t erased_first = 100;
  constexpr unsigned erased_value = 0xff;
  constexpr unsigned last_pattern = 0x7a;
  require(image.bytes[0] == first_pattern && image.known[0] == 1 &&
              image.bytes[erased_first] == erased_value && image.known[erased_first] == 1 &&
              image.bytes.back() == last_pattern && image.known.back() == 1 && image.known[1] == 0,
          "Exact bytes and bit-to-byte knowledge decoding");
  constexpr unsigned erased_word = 50;
  require(image.erased_words[erased_word / 8] == (1U << (erased_word % 8)),
          "Word-program eligibility is retained");
  require(image.encode() == bytes, "Byte-exact canonical image round trip");
  bytes.back() ^= 1;
  bool rejected = false;
  try {
    static_cast<void>(blofeld::vi::FlashImage::decode(bytes));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "CRC corruption must not become a device image");
  auto invalid = image;
  invalid.bytes[1] = 1;
  rejected = false;
  try {
    static_cast<void>(invalid.encode());
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Unknown bytes cannot contain invented values");
  invalid = image;
  invalid.bytes[erased_first] = 0;
  rejected = false;
  try {
    static_cast<void>(invalid.encode());
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Erase metadata must describe known erased words");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
