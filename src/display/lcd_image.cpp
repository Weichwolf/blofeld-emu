// SPDX-License-Identifier: GPL-3.0-only
#include "display/lcd_image.hpp"

#include "common/bus.hpp"
#include "display/st7565.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld {
namespace {
void validate(const LcdPanel panel) {
  if (panel.width == 0 || panel.width > St7565::columns || panel.first_segment >= St7565::columns) {
    throw Fault("Invalid LCD panel width or segment origin");
  }
  const auto available =
      panel.reverse_segments ? panel.first_segment + 1 : St7565::columns - panel.first_segment;
  if (panel.width > available) { throw Fault("LCD panel exceeds its declared segment range"); }
}

struct Position {
  unsigned x;
  unsigned y;
};

LcdPixel sample(const St7565 &lcd, const LcdPanel panel, const Position position) {
  const auto [x, y] = position;
  const auto state = lcd.state();
  if (!state.display_on) { return LcdPixel::off; }
  if (state.all_points) { return LcdPixel::on; }
  const auto segment = panel.reverse_segments ? panel.first_segment - x : panel.first_segment + x;
  const auto column = state.adc_reverse ? St7565::columns - 1 - segment : segment;
  const auto common = panel.reverse_commons ? LcdImage::height - 1 - y : y;
  const auto offset = state.common_reverse ? LcdImage::height - 1 - common : common;
  const auto line = (state.start_line + offset) % LcdImage::height;
  const auto byte = lcd.ram(line / 8, column);
  const auto bit = 1U << (line % 8);
  if ((byte.known & bit) == 0) { return LcdPixel::unknown; }
  const bool lit = ((byte.value & bit) != 0) != state.reverse;
  return lit ? LcdPixel::on : LcdPixel::off;
}
}

LcdImage::LcdImage(const unsigned width)
    : width_(width), pixels_(static_cast<std::size_t>(width) * height, LcdPixel::unknown) {}

LcdPixel LcdImage::pixel(const unsigned x, const unsigned y) const {
  if (x >= width_ || y >= height) { throw Fault("LCD image pixel is outside the captured frame"); }
  return pixels_.at((static_cast<std::size_t>(y) * width_) + x);
}

LcdImage capture_lcd(const St7565 &controller, const LcdPanel panel) {
  validate(panel);
  const auto frozen = controller;
  // Establish reset even if a later display mode forces a constant image.
  static_cast<void>(frozen.state());
  LcdImage image(panel.width);
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    for (unsigned x = 0; x < panel.width; ++x) {
      image.pixels_.at((static_cast<std::size_t>(y) * panel.width) + x) =
          sample(frozen, panel, {.x = x, .y = y});
    }
  }
  return image;
}

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
