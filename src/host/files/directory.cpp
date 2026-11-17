// SPDX-License-Identifier: GPL-3.0-only
#include "files/directory.hpp"

#include "files/read.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#if defined(__APPLE__)
#include <sys/_types/_mode_t.h>
#include <sys/_types/_ssize_t.h>
#include <sys/fcntl.h>
#include <sys/stdio.h>
#include <sys/unistd.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <sys/syscall.h>
#include <sys/types.h>
#endif
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>

namespace blofeld::host {
namespace {
constexpr auto owner_permissions = 0600;

[[noreturn]] void fail(const std::string_view operation, const std::string_view path = {}) {
  const auto error = errno;
  throw std::system_error(
      error, std::generic_category(), std::string(operation) + ": " + std::string(path));
}

std::string entry_name(const std::string_view name) {
  if (name.empty() || name == "." || name == ".." || name.contains('/') || name.contains('\0')) {
    throw std::invalid_argument("Publication requires a single filename");
  }
  return std::string(name);
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

void sync_file(const int descriptor, const std::string_view filename) {
  while (::fsync(descriptor) != 0) {
    if (errno != EINTR) { fail("Sync publication", filename); }
  }
}

void check_revision(const int directory, const std::string &name, const FileRevision &expected) {
  struct stat status{};
  if (::fstatat(directory, name.c_str(), &status, AT_SYMLINK_NOFOLLOW) != 0) {
    fail("Inspect publication target", name);
  }
  if (!S_ISREG(status.st_mode) || FileRevision::from(status) != expected) {
    throw std::runtime_error("Publication target changed since startup: " + name);
  }
}
}

Directory::Directory(const std::filesystem::path &path, const Lock lock)
    : descriptor_(::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW)) {
  if (descriptor_ < 0) { fail("Open publication directory", path.native()); }
  if (::flock(descriptor_, LOCK_EX | (lock == Lock::wait ? 0 : LOCK_NB)) != 0) {
    const auto error = errno;
    ::close(descriptor_);
    throw std::system_error(error, std::generic_category(), "Lock publication directory");
  }
}

Directory::~Directory() {
  ::close(descriptor_);
}

Presence::~Presence() {
  ::close(descriptor_);
}

Presence Directory::presence(const std::string_view name) const {
  const auto filename = entry_name(name);
  remove(filename);
  if (::mkfifoat(descriptor_, filename.c_str(), owner_permissions) != 0) {
    fail("Create presence", filename);
  }
  constexpr auto flags = O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW;
  const auto reader = ::openat(descriptor_, filename.c_str(), O_RDONLY | flags);
  if (reader < 0) { fail("Open presence reader", filename); }
  const auto writer = ::openat(descriptor_, filename.c_str(), O_WRONLY | flags);
  const auto error = errno;
  ::close(reader);
  if (writer < 0) {
    throw std::system_error(error, std::generic_category(), "Open presence writer " + filename);
  }
  return Presence(writer);
}

void Directory::replace(const std::string_view name,
                        const std::span<const std::uint8_t> bytes) const {
  publish(name, bytes, nullptr);
}

void Directory::commit(const std::string_view name,
                       const std::span<const std::uint8_t> bytes,
                       const FileRevision &expected) const {
  publish(name, bytes, &expected);
}

void Directory::publish(const std::string_view name,
                        const std::span<const std::uint8_t> bytes,
                        const FileRevision *const expected) const {
  const auto filename = entry_name(name);
  const auto staging = expected != nullptr ? ".blofeld-state." + std::to_string(::getpid())
                                           : "." + filename + ".tmp";
  if (expected != nullptr) {
    check_revision(descriptor_, filename, *expected);
    if (same_contents(descriptor_, filename, bytes, *expected)) {
      check_revision(descriptor_, filename, *expected);
      return;
    }
  } else if (::unlinkat(descriptor_, staging.c_str(), 0) != 0 && errno != ENOENT) {
    fail("Remove stale publication", staging);
  }
  const auto output = ::openat(descriptor_,
                               staging.c_str(),
                               O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                               owner_permissions);
  if (output < 0) { fail("Create publication", staging); }
  try {
    write_all(output, bytes);
    if (expected != nullptr) {
      if (::fchmod(output, static_cast<mode_t>(expected->permissions)) != 0) {
        fail("Set publication permissions", staging);
      }
      sync_file(output, staging);
    }
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
  try {
    if (expected != nullptr) { check_revision(descriptor_, filename, *expected); }
#if defined(__APPLE__)
    const auto replaced = ::renameat(descriptor_, staging.c_str(), descriptor_, filename.c_str());
#else
    const auto replaced =
        ::syscall(SYS_renameat, descriptor_, staging.c_str(), descriptor_, filename.c_str());
#endif
    if (replaced != 0) { fail("Replace publication", filename); }
  } catch (...) {
    ::unlinkat(descriptor_, staging.c_str(), 0);
    throw;
  }
  if (expected != nullptr) { sync_file(descriptor_, filename); }
}

void Directory::remove(const std::string_view name) const {
  const auto filename = entry_name(name);
  if (::unlinkat(descriptor_, filename.c_str(), 0) != 0 && errno != ENOENT) {
    fail("Remove publication", filename);
  }
}
}
