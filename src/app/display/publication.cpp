// SPDX-License-Identifier: GPL-3.0-only
#include "display/publication.hpp"

#include "display/lcd_image.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace blofeld::app {
std::vector<std::uint8_t> encode_pbm(const vi::LcdImage &image) {
  const auto header = "P4\n" + std::to_string(image.width()) + " 64\n";
  std::vector<std::uint8_t> bytes(header.begin(), header.end());
  const auto stride = (image.width() + 7) / 8;
  for (unsigned y = 0; y < vi::LcdImage::height; ++y) {
    for (unsigned group = 0; group < stride; ++group) {
      unsigned packed = 0;
      for (unsigned bit = 0; bit < 8; ++bit) {
        const auto x = (group * 8) + bit;
        if (x >= image.width()) { continue; }
        const auto pixel = image.pixel(x, y);
        if (pixel == vi::LcdPixel::unknown) {
          throw std::runtime_error("Unknown LCD pixel x=" + std::to_string(x) +
                                   " y=" + std::to_string(y));
        }
        if (pixel == vi::LcdPixel::on) { packed |= 1U << (7 - bit); }
      }
      bytes.push_back(static_cast<std::uint8_t>(packed));
    }
  }
  return bytes;
}

std::string encode_text(const vi::LcdImage &image) {
  std::string text;
  text.reserve(static_cast<std::size_t>(image.width() + 1) * vi::LcdImage::height);
  for (unsigned y = 0; y < vi::LcdImage::height; ++y) {
    for (unsigned x = 0; x < image.width(); ++x) {
      switch (image.pixel(x, y)) {
        case vi::LcdPixel::off: text.push_back(' '); break;
        case vi::LcdPixel::on: text.push_back('#'); break;
        case vi::LcdPixel::unknown: text.push_back('?'); break;
        default: throw std::runtime_error("Invalid LCD pixel");
      }
    }
    text.push_back('\n');
  }
  return text;
}

DisplayPublication::DisplayPublication(const std::filesystem::path &directory)
    : directory_(directory) {}

bool DisplayPublication::publish(const vi::LcdImage &image) {
  if (previous_ == image) { return false; }
  previous_.reset();
  const auto text = encode_text(image);
  if (text.contains('?')) {
    directory_.remove("display.pbm");
  } else {
    directory_.replace("display.pbm", encode_pbm(image));
  }
  directory_.replace("display.txt",
                     {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
  previous_ = image;
  return true;
}
}
