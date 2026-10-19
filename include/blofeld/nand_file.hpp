// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/nand_image.hpp"
#include "blofeld/private_file.hpp"

#include <filesystem>

namespace blofeld {
using NandFilePhase = PrivateFilePhase;
using NandFileObserver = PrivateFileObserver;

[[nodiscard]] NandImage load_nand_image(const std::filesystem::path &path);
void save_nand_image(const std::filesystem::path &path,
                     const NandImage &image,
                     const NandFileObserver &observer = {});
}
