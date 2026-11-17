// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

struct stat;

namespace blofeld::host {
struct FileRevision {
  std::uintmax_t device = 0, inode = 0;
  std::intmax_t size = 0, modified_seconds = 0, modified_nanoseconds = 0, changed_seconds = 0,
                changed_nanoseconds = 0;
  unsigned permissions = 0;

  [[nodiscard]] static FileRevision from(const struct stat &status) noexcept;
  [[nodiscard]] bool operator==(const FileRevision &) const = default;
};

// Read one bounded regular file without following its final symlink. Changed,
// truncated or oversized inputs fail rather than returning a partial snapshot.
[[nodiscard]] std::vector<std::uint8_t> read_file(const std::filesystem::path &path,
                                                  std::size_t maximum_bytes,
                                                  FileRevision *revision = nullptr);
[[nodiscard]] bool same_contents(int directory,
                                 const std::filesystem::path &name,
                                 std::span<const std::uint8_t> bytes,
                                 const FileRevision &expected);
}
