// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
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
  if (before.st_size != after.st_size || before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
      before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
      before.st_ctim.tv_sec != after.st_ctim.tv_sec ||
      before.st_ctim.tv_nsec != after.st_ctim.tv_nsec) {
    throw std::runtime_error("Input file changed while reading: " + path.string());
  }
  return bytes;
}
}
