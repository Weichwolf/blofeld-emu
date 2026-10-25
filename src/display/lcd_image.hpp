// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace blofeld {
// Explicit 64-COM panel wiring; no physical Blofeld mapping is inferred.
struct LcdPanel {
  unsigned width = 0;
  unsigned first_segment = 0;
  bool reverse_segments = false;
  bool reverse_commons = false;
};

enum class LcdPixel : std::uint8_t { off, on, unknown };

class LcdImage {
public:
  static constexpr unsigned height = 64;

  [[nodiscard]] unsigned width() const { return width_; }

  [[nodiscard]] LcdPixel pixel(unsigned x, unsigned y) const;
  bool operator==(const LcdImage &) const = default;

private:
  explicit LcdImage(unsigned width);
  unsigned width_;
  std::vector<LcdPixel> pixels_;
  friend LcdImage capture_lcd(const St7565 &controller, LcdPanel panel);
};

// Caller supplies a stable, inactive board boundary. This function copies the
// controller and inspects RAM only; it never performs a native display read.
[[nodiscard]] LcdImage capture_lcd(const St7565 &controller, LcdPanel panel);
// P4: row-major, MSB first, black=1. Unknown pixels reject instead of becoming 0.
[[nodiscard]] std::vector<std::uint8_t> encode_lcd_pbm(const LcdImage &image);
// Diagnostic ASCII raster: '#' on, space off, '?' unknown, newline per row.
[[nodiscard]] std::string encode_lcd_text(const LcdImage &image);
// Optional saved screenshot: 1-bit grayscale PNG, not the live publication path.
[[nodiscard]] std::vector<std::uint8_t> encode_lcd_png(const LcdImage &image);
}
