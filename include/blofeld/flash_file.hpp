// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/flash_image.hpp"
#include "blofeld/private_file.hpp"

#include <filesystem>

namespace blofeld {

// Host file transaction observations for tracing and controlled process-death
// tests. These do not describe the MCU's electrical erase/program phases.
using FlashFilePhase = PrivateFilePhase;
using FlashFileObserver = PrivateFileObserver;

[[nodiscard]] McuFlashImage load_flash_image(const std::filesystem::path &path);
void save_flash_image(const std::filesystem::path &path,
                      const McuFlashImage &image,
                      const FlashFileObserver &observer = {});

}
