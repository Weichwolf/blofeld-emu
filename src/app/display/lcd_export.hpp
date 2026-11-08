// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/private_file.hpp"
#include "vi/display/lcd_image.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace blofeld {
// P4: row-major, MSB first, black=1. Unknown pixels reject instead of becoming 0.
[[nodiscard]] std::vector<std::uint8_t> encode_lcd_pbm(const LcdImage &image);
// Diagnostic ASCII raster: '#' on, space off, '?' unknown, newline per row.
[[nodiscard]] std::string encode_lcd_text(const LcdImage &image);
// Optional saved screenshot: 1-bit grayscale PNG, not the live publication path.
[[nodiscard]] std::vector<std::uint8_t> encode_lcd_png(const LcdImage &image);

// Host-only I/O, outside emulated hardware and any realtime audio callback.
// The directory must exist. Stable identity and one-writer lease are retained.
class LcdPublisher {
public:
  explicit LcdPublisher(const std::filesystem::path &directory);
  // Complete PBM and text files are replaced individually, not as one transaction.
  // Returns false without I/O when the previously published pixels are unchanged.
  bool publish(const LcdImage &image, const PrivateFileObserver &observer = {});

private:
  PrivateDirectory directory_;
  PrivateFileLease lease_;
  std::optional<LcdImage> previous_;
};
}
