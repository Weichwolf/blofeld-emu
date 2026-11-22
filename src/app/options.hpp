// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>

namespace blofeld::app {
struct Options {
  static constexpr std::uint64_t default_startup_watchdog_seconds = 600;
  std::filesystem::path profile;
  std::optional<std::filesystem::path> midi, wav;
  std::optional<std::int32_t> midi_source, midi_destination;
#if defined(__linux__)
  std::string alsa_device;
#endif
  std::uint8_t midi_group = 0;
  std::uint64_t tail_seconds = 5;
  std::uint64_t startup_watchdog_seconds = default_startup_watchdog_seconds;
};

[[nodiscard]] Options options(std::span<const char *const> arguments);
}
