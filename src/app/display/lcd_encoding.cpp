// SPDX-License-Identifier: GPL-3.0-only
#include "app/display/lcd_export.hpp"
#include "common/bus.hpp"
#include "vi/display/lcd_image.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld {
std::vector<std::uint8_t> encode_lcd_pbm(const LcdImage &image) {
  const auto header = "P4\n" + std::to_string(image.width()) + " 64\n";
  std::vector<std::uint8_t> bytes(header.begin(), header.end());
  const auto stride = (image.width() + 7) / 8;
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    for (unsigned group = 0; group < stride; ++group) {
      unsigned packed = 0;
      for (unsigned bit = 0; bit < 8; ++bit) {
        const auto x = (group * 8) + bit;
        if (x >= image.width()) { continue; }
        const auto pixel = image.pixel(x, y);
        if (pixel == LcdPixel::unknown) {
          throw Fault("Unknown LCD screenshot pixel x=" + std::to_string(x) +
                      " y=" + std::to_string(y));
        }
        if (pixel == LcdPixel::on) { packed |= 1U << (7 - bit); }
      }
      bytes.push_back(static_cast<std::uint8_t>(packed));
    }
  }
  return bytes;
}

std::string encode_lcd_text(const LcdImage &image) {
  std::string text;
  text.reserve(static_cast<std::size_t>(image.width() + 1) * LcdImage::height);
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    for (unsigned x = 0; x < image.width(); ++x) {
      switch (image.pixel(x, y)) {
        case LcdPixel::off: text.push_back(' '); break;
        case LcdPixel::on: text.push_back('#'); break;
        case LcdPixel::unknown: text.push_back('?'); break;
        default: throw Fault("Invalid captured LCD pixel");
      }
    }
    text.push_back('\n');
  }
  return text;
}
}
