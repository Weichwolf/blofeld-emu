// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "boot_rom.hpp"

#include <filesystem>

namespace blofeld::app {
struct Profile {
  std::filesystem::path firmware, boot_rom, device_state, storage, display;
  BootFormat boot_format = BootFormat::rom;
};

[[nodiscard]] Profile load_profile(const std::filesystem::path &path);
[[nodiscard]] std::filesystem::path default_profile();
}
