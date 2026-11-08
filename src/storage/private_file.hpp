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
class PrivateFile;

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
  friend class PrivateFile;
};

enum class PrivateFileAccess : std::uint8_t { read_only, read_write };

struct PrivateFileRead {
  std::uint64_t offset = 0;
  std::span<std::uint8_t> bytes;
};

struct PrivateFileAppend {
  std::uint64_t offset = 0;
  std::span<const std::uint8_t> pending;
  std::span<const std::uint8_t> commit;
};

// Stable open image descriptor. A committed prefix survives incomplete tails.
class PrivateFile {
public:
  PrivateFile(const PrivateDirectory &directory,
              const std::filesystem::path &filename,
              PrivateFileAccess access = PrivateFileAccess::read_only);
  ~PrivateFile();
  PrivateFile(const PrivateFile &) = delete;
  PrivateFile &operator=(const PrivateFile &) = delete;
  [[nodiscard]] std::uint64_t size() const;
  void read(PrivateFileRead range) const;
  void append(PrivateFileAppend range, const PrivateFileObserver &observer = {});

private:
  const PrivateDirectory &directory_;
  const std::filesystem::path filename_;
  int descriptor_ = -1;
  PrivateFileAccess access_;
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
