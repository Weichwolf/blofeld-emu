// SPDX-License-Identifier: GPL-3.0-only
#include "storage/private_file.hpp"

#include "common/bus.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

namespace blofeld {
namespace {
[[noreturn]] void file_error(const char *operation);

class Descriptor {
public:
  explicit Descriptor(const int value) : value_(value) {}

  ~Descriptor() {
    if (value_ >= 0) { static_cast<void>(close(value_)); }
  }

  Descriptor(const Descriptor &) = delete;
  Descriptor &operator=(const Descriptor &) = delete;

  [[nodiscard]] int get() const { return value_; }

  void close_checked() {
    const int descriptor = std::exchange(value_, -1);
    if (close(descriptor) < 0) { file_error("Cannot close synced private state file"); }
  }

private:
  int value_ = -1;
};

[[noreturn]] void file_error(const char *operation) {
  const int code = errno;
  throw Fault(std::string(operation) + ": " +
              std::error_code(code, std::generic_category()).message());
}

void observe(const PrivateFileObserver &observer, const PrivateFilePhase phase) {
  if (observer) { observer(phase); }
}

void write_all(const int descriptor, const std::span<const std::uint8_t> bytes) {
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = write(descriptor, bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { file_error("Cannot write temporary private state file"); }
    offset += static_cast<std::size_t>(count);
  }
}

std::string random_suffix() {
  std::array<std::uint8_t, 8> nonce{};
  std::size_t offset = 0;
  while (offset < nonce.size()) {
    const auto count = getrandom(nonce.data() + offset, nonce.size() - offset, GRND_NONBLOCK);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { file_error("Cannot obtain private state temporary-file nonce"); }
    offset += static_cast<std::size_t>(count);
  }
  constexpr std::string_view digits = "0123456789abcdef";
  std::string suffix;
  for (const auto byte : nonce) {
    suffix.push_back(digits[byte >> 4U]);
    suffix.push_back(digits[byte & 15U]);
  }
  return suffix;
}

struct Temporary {
  int descriptor = -1;
  std::string name;
};

Temporary create_temporary(const int directory, const std::string &filename) {
  constexpr unsigned attempts = 64;
  for (unsigned attempt = 0; attempt < attempts; ++attempt) {
    auto name = "." + filename + ".tmp." + random_suffix();
    const int descriptor = openat(directory,
                                  name.c_str(),
                                  O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                                  S_IRUSR | S_IWUSR);
    if (descriptor >= 0) { return {.descriptor = descriptor, .name = std::move(name)}; }
    if (errno != EEXIST) { file_error("Cannot create temporary private state file"); }
  }
  throw Fault("private state temporary-file nonce collisions exceeded the retry limit");
}

std::string local_name(const std::filesystem::path &filename) {
  const auto name = filename.string();
  if (name.empty() || filename.has_parent_path() || name == "." || name == ".." ||
      name.find('\0') != std::string::npos) {
    throw Fault("Private state directory requires a single local filename");
  }
  return name;
}
}

class PrivateDirectory::Impl {
public:
  explicit Impl(const std::filesystem::path &path)
      : directory(
            open(std::filesystem::canonical(path).c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC)) {
    if (directory.get() < 0) { file_error("Cannot open private state directory"); }
  }

  Descriptor directory;
};

PrivateDirectory::PrivateDirectory(const std::filesystem::path &path)
    : impl_(std::make_unique<Impl>(path)) {}

PrivateDirectory::~PrivateDirectory() = default;

bool PrivateDirectory::contains(const std::filesystem::path &filename) const {
  const auto name = local_name(filename);
  struct stat information{};
  if (fstatat(impl_->directory.get(), name.c_str(), &information, AT_SYMLINK_NOFOLLOW) == 0) {
    return true;
  }
  if (errno == ENOENT) { return false; }
  file_error("Cannot inspect private state directory entry");
}

class PrivateFileLease::Impl {
public:
  Impl(const int directory, const std::string &filename)
      : descriptor(openat(directory,
                          filename.c_str(),
                          O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK,
                          S_IRUSR | S_IWUSR)) {
    if (descriptor.get() < 0) { file_error("Cannot open private state writer lease"); }
    struct stat information{};
    if (fstat(descriptor.get(), &information) < 0) {
      file_error("Cannot inspect private state writer lease");
    }
    if (!S_ISREG(information.st_mode)) {
      throw Fault("Private state writer lease is not a regular file");
    }
    if (flock(descriptor.get(), LOCK_EX | LOCK_NB) < 0) {
      file_error("Cannot acquire private state writer lease");
    }
    if (fchmod(descriptor.get(), S_IRUSR | S_IWUSR) < 0) {
      file_error("Cannot set private state lease permissions");
    }
  }

  Descriptor descriptor;
};

PrivateFileLease::PrivateFileLease(const PrivateDirectory &directory,
                                   const std::filesystem::path &filename)
    : impl_(std::make_unique<Impl>(directory.impl_->directory.get(), local_name(filename))) {}

PrivateFileLease::~PrivateFileLease() = default;

std::vector<std::uint8_t> PrivateDirectory::read(const std::filesystem::path &filename,
                                                 const std::size_t maximum_bytes) const {
  const auto name = local_name(filename);
  const Descriptor input(
      openat(impl_->directory.get(), name.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC));
  if (input.get() < 0) { file_error("Cannot open private state input"); }
  struct stat information{};
  if (fstat(input.get(), &information) < 0) { file_error("Cannot inspect private state input"); }
  if (!S_ISREG(information.st_mode) || information.st_size < 0 ||
      static_cast<std::uint64_t>(information.st_size) > maximum_bytes) {
    throw Fault("Private state input type or length exceeds its bound: " + name);
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(information.st_size));
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = ::read(input.get(), bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && errno == EINTR) { continue; }
    if (count < 0) { file_error("Cannot read private state input"); }
    if (count == 0) { throw Fault("Private state input shrank during reading"); }
    offset += static_cast<std::size_t>(count);
  }
  std::uint8_t extra = 0;
  ssize_t count = 0;
  do { count = ::read(input.get(), &extra, 1); } while (count < 0 && errno == EINTR);
  if (count < 0) { file_error("Cannot read private state input ending"); }
  if (count != 0) { throw Fault("Private state input grew during reading"); }
  return bytes;
}

void PrivateDirectory::write(const std::filesystem::path &entry,
                             const std::span<const std::uint8_t> bytes,
                             const PrivateFileObserver &observer) const {
  const auto filename = local_name(entry);
  const auto &directory = impl_->directory;
  struct stat previous{};
  const int inspected = fstatat(directory.get(), filename.c_str(), &previous, AT_SYMLINK_NOFOLLOW);
  if (inspected == 0 && S_ISLNK(previous.st_mode)) {
    throw Fault("Refusing to replace a symlink with a private state file: " + filename);
  }
  if (inspected < 0 && errno != ENOENT) { file_error("Cannot inspect private state destination"); }
  const auto temporary = create_temporary(directory.get(), filename);
  Descriptor output(temporary.descriptor);
  try {
    if (fchmod(output.get(), S_IRUSR | S_IWUSR) < 0) {
      file_error("Cannot set private state permissions");
    }
    observe(observer, PrivateFilePhase::created);
    write_all(output.get(), bytes);
    observe(observer, PrivateFilePhase::written);
    if (fsync(output.get()) < 0) { file_error("Cannot sync temporary private state file"); }
    observe(observer, PrivateFilePhase::synced);
    output.close_checked();
    if (syscall(SYS_renameat,
                static_cast<long>(directory.get()),
                temporary.name.c_str(),
                static_cast<long>(directory.get()),
                filename.c_str()) < 0) {
      file_error("Cannot publish private state file");
    }
    observe(observer, PrivateFilePhase::published);
    if (fsync(directory.get()) < 0) { file_error("Cannot sync private state directory"); }
    observe(observer, PrivateFilePhase::directory_synced);
  } catch (...) {
    static_cast<void>(unlinkat(directory.get(), temporary.name.c_str(), 0));
    throw;
  }
}

std::vector<std::uint8_t> read_private_file(const std::filesystem::path &path,
                                            const std::size_t maximum_bytes) {
  const PrivateDirectory directory(path.parent_path().empty() ? std::filesystem::path(".")
                                                              : path.parent_path());
  return directory.read(path.filename(), maximum_bytes);
}

void write_private_file(const std::filesystem::path &path,
                        const std::span<const std::uint8_t> bytes,
                        const PrivateFileObserver &observer) {
  const PrivateDirectory directory(path.parent_path().empty() ? std::filesystem::path(".")
                                                              : path.parent_path());
  directory.write(path.filename(), bytes, observer);
}

}
