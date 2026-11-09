// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"

#include <algorithm>
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
  explicit File(const std::filesystem::path &path)
      : descriptor(::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW)) {
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

std::vector<std::uint8_t> read_file(const std::filesystem::path &path,
                                    const std::size_t maximum_bytes) {
  const File file(path);
  const auto before = information(file);
  if (!S_ISREG(before.st_mode) || before.st_size < 0 ||
      static_cast<std::uint64_t>(before.st_size) > maximum_bytes) {
    throw std::runtime_error("Input must be a regular file within its size limit: " +
                             path.string());
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(before.st_size));
  read_all(file, bytes);
  const auto after = information(file);
#if defined(__APPLE__)
  const auto &modified_before = before.st_mtimespec;
  const auto &modified_after = after.st_mtimespec;
  const auto &changed_before = before.st_ctimespec;
  const auto &changed_after = after.st_ctimespec;
#else
  const auto &modified_before = before.st_mtim;
  const auto &modified_after = after.st_mtim;
  const auto &changed_before = before.st_ctim;
  const auto &changed_after = after.st_ctim;
#endif
  if (before.st_size != after.st_size || modified_before.tv_sec != modified_after.tv_sec ||
      modified_before.tv_nsec != modified_after.tv_nsec ||
      changed_before.tv_sec != changed_after.tv_sec ||
      changed_before.tv_nsec != changed_after.tv_nsec) {
    throw std::runtime_error("Input file changed while reading: " + path.string());
  }
  return bytes;
}
}
