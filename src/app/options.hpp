// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>

namespace blofeld::app {
struct Options {
  std::filesystem::path profile;
  std::optional<std::filesystem::path> midi, wav;
  std::uint64_t tail_seconds = 5;
};

[[nodiscard]] Options options(std::span<const char *const> arguments);
}
