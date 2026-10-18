// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/flash_file.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <ios>
#include <span>
#include <string>
#include <string_view>
#include <sys/random.h>
#include <sys/stat.h>
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
    if (close(descriptor) < 0) { file_error("Cannot close synced MCU NVM file"); }
  }

private:
  int value_ = -1;
};

[[noreturn]] void file_error(const char *operation) {
  const int code = errno;
  throw Fault(std::string(operation) + ": " +
              std::error_code(code, std::generic_category()).message());
}

void extension(const std::filesystem::path &path) {
  if (path.extension() != ".bfnvm") {
    throw Fault("MCU NVM files require the private .bfnvm extension");
  }
}

void observe(const FlashFileObserver &observer, const FlashFilePhase phase) {
  if (observer) { observer(phase); }
}

void write_all(const int descriptor, const std::span<const std::uint8_t> bytes) {
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = write(descriptor, bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { file_error("Cannot write temporary MCU NVM file"); }
    offset += static_cast<std::size_t>(count);
  }
}

std::string random_suffix() {
  std::array<std::uint8_t, 8> nonce{};
  std::size_t offset = 0;
  while (offset < nonce.size()) {
    const auto count = getrandom(nonce.data() + offset, nonce.size() - offset, GRND_NONBLOCK);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { file_error("Cannot obtain MCU NVM temporary-file nonce"); }
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
    if (errno != EEXIST) { file_error("Cannot create temporary MCU NVM file"); }
  }
  throw Fault("MCU NVM temporary-file nonce collisions exceeded the retry limit");
}
}

McuFlashImage load_flash_image(const std::filesystem::path &path) {
  extension(path);
  if (!std::filesystem::is_regular_file(path)) {
    throw Fault("MCU NVM input is not a regular file: " + path.string());
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) { throw Fault("Cannot open MCU NVM file: " + path.string()); }
  std::array<char, flash_image_file_bytes + 1> buffer{};
  file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
  if (file.bad() || file.gcount() != static_cast<std::streamsize>(flash_image_file_bytes)) {
    throw Fault("Cannot read exact MCU NVM file length: " + path.string());
  }
  std::vector<std::uint8_t> bytes;
  bytes.reserve(flash_image_file_bytes);
  for (std::size_t index = 0; index < flash_image_file_bytes; ++index) {
    bytes.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(buffer[index])));
  }
  return decode_flash_image(bytes);
}

void save_flash_image(const std::filesystem::path &path,
                      const McuFlashImage &image,
                      const FlashFileObserver &observer) {
  extension(path);
  const auto bytes = encode_flash_image(image);
  const auto parent = std::filesystem::canonical(
      path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path());
  const Descriptor directory(open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
  if (directory.get() < 0) { file_error("Cannot open MCU NVM directory"); }
  const auto filename = path.filename().string();
  struct stat previous{};
  const int inspected = fstatat(directory.get(), filename.c_str(), &previous, AT_SYMLINK_NOFOLLOW);
  if (inspected == 0 && S_ISLNK(previous.st_mode)) {
    throw Fault("Refusing to replace a symlink with an MCU NVM file: " + path.string());
  }
  if (inspected < 0 && errno != ENOENT) { file_error("Cannot inspect MCU NVM destination"); }
  const auto temporary = create_temporary(directory.get(), filename);
  Descriptor output(temporary.descriptor);
  try {
    if (fchmod(output.get(), S_IRUSR | S_IWUSR) < 0) {
      file_error("Cannot set private MCU NVM permissions");
    }
    observe(observer, FlashFilePhase::created);
    write_all(output.get(), bytes);
    observe(observer, FlashFilePhase::written);
    if (fsync(output.get()) < 0) { file_error("Cannot sync temporary MCU NVM file"); }
    observe(observer, FlashFilePhase::synced);
    output.close_checked();
    std::error_code error;
    std::filesystem::rename(parent / temporary.name, parent / filename, error);
    if (error) { throw Fault("Cannot publish MCU NVM file: " + error.message()); }
    observe(observer, FlashFilePhase::published);
    if (fsync(directory.get()) < 0) { file_error("Cannot sync MCU NVM directory"); }
    observe(observer, FlashFilePhase::directory_synced);
  } catch (...) {
    static_cast<void>(unlinkat(directory.get(), temporary.name.c_str(), 0));
    throw;
  }
}

}
