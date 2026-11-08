// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <filesystem>

namespace blofeld::host {
// Resolve the process's inherited configuration root during single-threaded
// startup, before creating workers or changing environment entries.
[[nodiscard]] std::filesystem::path configuration_home();
}
