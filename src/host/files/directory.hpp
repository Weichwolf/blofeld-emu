// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string_view>

namespace blofeld::host {
// Single-writer directory for atomic publication. The open identity survives
// path renames. No instrument types, serialization or device timing belong here.
class Directory {
public:
  explicit Directory(const std::filesystem::path &path);
  ~Directory();
  Directory(const Directory &) = delete;
  Directory &operator=(const Directory &) = delete;

  // Replace one complete directory entry. Transient publication only; this
  // operation makes no durability guarantee across power loss.
  void replace(std::string_view name, std::span<const std::uint8_t> bytes) const;
  void remove(std::string_view name) const;

private:
  int descriptor_ = -1;
};
}
