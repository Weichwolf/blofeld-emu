// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/lcd_export.hpp"

#include "blofeld/lcd_image.hpp"
#include "blofeld/private_file.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

namespace blofeld {
LcdPublisher::LcdPublisher(const std::filesystem::path &directory)
    : directory_(directory), lease_(directory_, ".display.lock") {}

bool LcdPublisher::publish(const LcdImage &image, const PrivateFileObserver &observer) {
  if (previous_ == image) { return false; }
  const auto bitmap = encode_lcd_pbm(image);
  const auto text = encode_lcd_text(image);
  const std::vector<std::uint8_t> text_bytes(text.begin(), text.end());
  // A failure after either rename may expose a different complete prefix.
  // Invalidate before I/O so returning to the old image also repairs both files.
  previous_.reset();
  directory_.write("display.pbm", bitmap, observer);
  directory_.write("display.txt", text_bytes, observer);
  previous_ = image;
  return true;
}
}
