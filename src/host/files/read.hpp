// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace blofeld::host {
// Read one bounded regular file without following its final symlink. Changed,
// truncated or oversized inputs fail rather than returning a partial snapshot.
[[nodiscard]] std::vector<std::uint8_t> read_file(const std::filesystem::path &path,
                                                  std::size_t maximum_bytes);
}
