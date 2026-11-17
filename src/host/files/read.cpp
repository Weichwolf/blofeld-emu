// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <sys/stat.h>
#if defined(__APPLE__)
#include <sys/_types/_ssize_t.h>
#include <sys/fcntl.h>
#else
#include <fcntl.h>
#include <sys/types.h>
#endif
#include <system_error>
#include <unistd.h>
#include <vector>

namespace blofeld::host {
namespace {
struct File {
  explicit File(const std::filesystem::path &path, const int directory = AT_FDCWD)
      : descriptor(
            ::openat(directory, path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW)) {
    if (descriptor < 0) {
      const auto error = errno;
      throw std::system_error(error, std::generic_category(), "Open input " + path.string());
    }
  }

  ~File() { ::close(descriptor); }

  File(const File &) = delete;
  File &operator=(const File &) = delete;
  int descriptor;
};

struct stat information(const File &file) {
  struct stat result{};
  if (::fstat(file.descriptor, &result) != 0) {
    throw std::system_error(errno, std::generic_category(), "Inspect input file");
  }
  return result;
}

void read_all(const File &file, std::span<std::uint8_t> bytes) {
  while (!bytes.empty()) {
    const auto count =
        std::min(bytes.size(), static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
    const auto received = ::read(file.descriptor, bytes.data(), count);
    if (received < 0) {
      if (errno == EINTR) { continue; }
      throw std::system_error(errno, std::generic_category(), "Read input file");
    }
    if (received == 0) { throw std::runtime_error("Input file was truncated while reading"); }
    bytes = bytes.subspan(static_cast<std::size_t>(received));
  }
}
}

FileRevision FileRevision::from(const struct stat &status) noexcept {
  constexpr unsigned permission_bits = 0777;
#if defined(__APPLE__)
  const auto &modified = status.st_mtimespec;
  const auto &changed = status.st_ctimespec;
#else
  const auto &modified = status.st_mtim;
  const auto &changed = status.st_ctim;
#endif
  return {.device = static_cast<std::uintmax_t>(status.st_dev),
          .inode = status.st_ino,
          .size = status.st_size,
          .modified_seconds = modified.tv_sec,
          .modified_nanoseconds = modified.tv_nsec,
          .changed_seconds = changed.tv_sec,
          .changed_nanoseconds = changed.tv_nsec,
          .permissions = status.st_mode & permission_bits};
}

std::vector<std::uint8_t> read_file(const std::filesystem::path &path,
                                    const std::size_t maximum_bytes,
                                    FileRevision *const revision) {
  const File file(path);
  const auto before = information(file);
  if (!S_ISREG(before.st_mode) || before.st_size < 0 ||
      static_cast<std::uint64_t>(before.st_size) > maximum_bytes) {
    throw std::runtime_error("Input must be a regular file within its size limit: " +
                             path.string());
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(before.st_size));
  read_all(file, bytes);
  const auto after = FileRevision::from(information(file));
  if (FileRevision::from(before) != after) {
    throw std::runtime_error("Input file changed while reading: " + path.string());
  }
  if (revision != nullptr) { *revision = after; }
  return bytes;
}

bool same_contents(const int directory,
                   const std::filesystem::path &name,
                   std::span<const std::uint8_t> bytes,
                   const FileRevision &expected) {
  if (expected.size < 0 || bytes.size() != static_cast<std::uintmax_t>(expected.size)) {
    return false;
  }
  const File file(name, directory);
  const auto verify = [&] {
    const auto status = information(file);
    if (!S_ISREG(status.st_mode) || FileRevision::from(status) != expected) {
      throw std::runtime_error("Publication target changed since startup: " + name.string());
    }
  };
  verify();
  constexpr auto block_size = std::size_t{64} * 1024;
  std::array<std::uint8_t, block_size> block;
  while (!bytes.empty()) {
    const auto count = std::min(bytes.size(), block.size());
    const auto input = std::span(block).first(count);
    read_all(file, input);
    if (!std::ranges::equal(input, bytes.first(count))) { return false; }
    bytes = bytes.subspan(count);
  }
  verify();
  return true;
}
}
