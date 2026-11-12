// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <filesystem>
#include <string_view>

namespace blofeld::host {
// Resolve the process's inherited configuration root during single-threaded
// startup, before creating workers or changing environment entries.
[[nodiscard]] std::string_view inherited_setting(std::string_view name);
[[nodiscard]] std::filesystem::path configuration_home();
[[nodiscard]] std::filesystem::path runtime_home();
}
