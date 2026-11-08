// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/firmware.hpp"

#include <filesystem>

namespace blofeld::app {
// Load the unchanged 1.25 MIDI update, directly or from its vendor ZIP.
[[nodiscard]] vi::FirmwareUpdate load_firmware(const std::filesystem::path &path);
}
