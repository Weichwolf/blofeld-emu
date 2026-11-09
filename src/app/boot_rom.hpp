// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace blofeld::app {
enum class BootFormat : std::uint8_t { rom, spi_prefix };

[[nodiscard]] std::vector<std::uint32_t> load_boot_rom(const std::filesystem::path &path,
                                                       BootFormat format);
}
