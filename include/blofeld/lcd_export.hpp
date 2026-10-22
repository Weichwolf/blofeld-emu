// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/lcd_image.hpp"
#include "blofeld/private_file.hpp"

#include <filesystem>
#include <optional>

namespace blofeld {
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
