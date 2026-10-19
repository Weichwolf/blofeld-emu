// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace blofeld {

// Host file phases, not physical MCU/NAND erase/program phases.
enum class PrivateFilePhase : std::uint8_t {
  created,
  written,
  synced,
  published,
  directory_synced
};
using PrivateFileObserver = std::function<void(PrivateFilePhase)>;

class PrivateFileLease;

// A stable directory identity shared by reads, replacement and writer leases.
// Namespace renames do not redirect later operations to a replacement path.
class PrivateDirectory {
public:
  explicit PrivateDirectory(const std::filesystem::path &path);
  ~PrivateDirectory();
  PrivateDirectory(const PrivateDirectory &) = delete;
  PrivateDirectory &operator=(const PrivateDirectory &) = delete;
  [[nodiscard]] bool contains(const std::filesystem::path &filename) const;
  [[nodiscard]] std::vector<std::uint8_t> read(const std::filesystem::path &filename,
                                               std::size_t maximum_bytes) const;
  void write(const std::filesystem::path &entry,
             std::span<const std::uint8_t> bytes,
             const PrivateFileObserver &observer = {}) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  friend class PrivateFileLease;
};

class PrivateFileLease {
public:
  PrivateFileLease(const PrivateDirectory &directory, const std::filesystem::path &filename);
  ~PrivateFileLease();
  PrivateFileLease(const PrivateFileLease &) = delete;
  PrivateFileLease &operator=(const PrivateFileLease &) = delete;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::vector<std::uint8_t> read_private_file(const std::filesystem::path &path,
                                                          std::size_t maximum_bytes);
void write_private_file(const std::filesystem::path &path,
                        std::span<const std::uint8_t> bytes,
                        const PrivateFileObserver &observer = {});

}
