// SPDX-License-Identifier: GPL-3.0-only
#include "files/directory.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/file.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>

namespace blofeld::host {
namespace {
[[noreturn]] void fail(const std::string_view operation, const std::string_view path = {}) {
  const auto error = errno;
  throw std::system_error(
      error, std::generic_category(), std::string(operation) + ": " + std::string(path));
}

void write_all(const int descriptor, std::span<const std::uint8_t> bytes) {
  while (!bytes.empty()) {
    const auto count =
        std::min(bytes.size(), static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
    const auto written = ::write(descriptor, bytes.data(), count);
    if (written < 0) {
      if (errno == EINTR) { continue; }
      fail("Write publication");
    }
    if (written == 0) { throw std::runtime_error("Publication write made no progress"); }
    bytes = bytes.subspan(static_cast<std::size_t>(written));
  }
}
}

Directory::Directory(const std::filesystem::path &path)
    : descriptor_(::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW)) {
  if (descriptor_ < 0) { fail("Open publication directory", path.native()); }
  if (::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
    const auto error = errno;
    ::close(descriptor_);
    throw std::system_error(error, std::generic_category(), "Lock publication directory");
  }
}

Directory::~Directory() {
  ::close(descriptor_);
}

void Directory::replace(const std::string_view name,
                        const std::span<const std::uint8_t> bytes) const {
  if (name.empty() || name == "." || name == ".." || name.contains('/') || name.contains('\0')) {
    throw std::invalid_argument("Publication requires a single filename");
  }
  const std::string filename(name);
  const auto staging = "." + filename + ".tmp";
  if (::unlinkat(descriptor_, staging.c_str(), 0) != 0 && errno != ENOENT) {
    fail("Remove stale publication", staging);
  }
  const auto output = ::openat(
      descriptor_, staging.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (output < 0) { fail("Create publication", staging); }
  try {
    write_all(output, bytes);
  } catch (...) {
    ::close(output);
    ::unlinkat(descriptor_, staging.c_str(), 0);
    throw;
  }
  if (::close(output) != 0) {
    const auto error = errno;
    ::unlinkat(descriptor_, staging.c_str(), 0);
    throw std::system_error(error, std::generic_category(), "Close publication " + staging);
  }
  if (::syscall(SYS_renameat, descriptor_, staging.c_str(), descriptor_, filename.c_str()) != 0) {
    const auto error = errno;
    ::unlinkat(descriptor_, staging.c_str(), 0);
    throw std::system_error(error, std::generic_category(), "Replace publication " + filename);
  }
}
}
