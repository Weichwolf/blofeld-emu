// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/flash_image.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>

namespace blofeld {

// Host file transaction observations for tracing and controlled process-death
// tests. These do not describe the MCU's electrical erase/program phases.
enum class FlashFilePhase : std::uint8_t { created, written, synced, published, directory_synced };
using FlashFileObserver = std::function<void(FlashFilePhase)>;

[[nodiscard]] McuFlashImage load_flash_image(const std::filesystem::path &path);
void save_flash_image(const std::filesystem::path &path,
                      const McuFlashImage &image,
                      const FlashFileObserver &observer = {});

}
