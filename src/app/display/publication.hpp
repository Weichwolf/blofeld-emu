// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/lcd_image.hpp"
#include "files/directory.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace blofeld::app {
[[nodiscard]] std::vector<std::uint8_t> encode_pbm(const vi::LcdImage &image);

// Consumes independent images on the app thread, outside core/audio execution.
class DisplayPublication {
public:
  explicit DisplayPublication(const std::filesystem::path &directory);
  // Unknown pixels revoke the bitmap. Unchanged images perform no I/O.
  bool publish(const vi::LcdImage &image);

private:
  host::Directory directory_;
  host::Presence presence_;
  std::optional<vi::LcdImage> previous_;
};
}
