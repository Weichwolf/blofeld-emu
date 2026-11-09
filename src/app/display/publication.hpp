// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/lcd_image.hpp"
#include "files/directory.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::app {
[[nodiscard]] std::vector<std::uint8_t> encode_pbm(const vi::LcdImage &image);
[[nodiscard]] std::string encode_text(const vi::LcdImage &image);

// Consumes independent images on the app thread, outside core/audio execution.
class DisplayPublication {
public:
  explicit DisplayPublication(const std::filesystem::path &directory);
  // PBM/text are individually atomic. Unknown pixels remain explicit in text;
  // the bitmap is removed instead of retaining a misleading previous image.
  // Unchanged images perform no I/O; errors invalidate that comparison.
  bool publish(const vi::LcdImage &image);

private:
  host::Directory directory_;
  std::optional<vi::LcdImage> previous_;
};
}
