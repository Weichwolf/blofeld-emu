// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/sample_library.hpp"

#include <filesystem>
#include <iosfwd>
#include <string_view>

namespace blofeld::samples {
void wave_information(const std::filesystem::path &path, std::ostream &output);
void append_wave(Program &program,
                 const std::filesystem::path &path,
                 std::string_view root,
                 unsigned highest);
}
