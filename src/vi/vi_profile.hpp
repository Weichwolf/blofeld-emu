// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "vi/vi.hpp"

#include <filesystem>

namespace blofeld::vi {
struct Profile {
  std::filesystem::path firmware;
  std::filesystem::path boot_rom;
  std::filesystem::path device_state;
  std::filesystem::path nand;
  std::filesystem::path display;
};

inline constexpr std::uint64_t timebase_hz = 635040000000;
inline constexpr std::uint32_t sample_rate = 44100;

[[nodiscard]] Profile load_profile(const std::filesystem::path &path);
[[nodiscard]] Config vi_config(const Profile &profile);
}
