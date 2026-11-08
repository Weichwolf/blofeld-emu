// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"

#include <cstdint>
#include <vector>

namespace blofeld::vi {
// Visible segment range and orientation; supplied by the instrument binding.
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

// Capture on the controller owner. The returned image is independent and may
// be handed to the app thread; inspection does not perform firmware reads.
[[nodiscard]] LcdImage capture_lcd(const St7565 &controller, LcdPanel panel);
}
