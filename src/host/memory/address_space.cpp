// SPDX-License-Identifier: GPL-3.0-only
#include "memory/address_space.hpp"

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <span>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace blofeld::host::memory {
namespace {
constexpr int anonymous = MAP_PRIVATE | MAP_ANONYMOUS
#ifdef MAP_NORESERVE
                          | MAP_NORESERVE
#endif
    ;

[[noreturn]] void failed(const char *operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}

std::size_t page_size(const std::size_t bytes) {
  const auto value = sysconf(_SC_PAGESIZE);
  if (value <= 0) { throw std::runtime_error("Host page size is unavailable"); }
  const auto page = static_cast<std::size_t>(value);
  if (bytes == 0 || bytes % page != 0) {
    throw std::invalid_argument("Mapped memory requires a nonempty whole-page extent");
  }
  return page;
}

int shared_file() {
  static std::atomic<std::uint32_t> sequence{0};
  for (;;) {
    const auto name = "/vi-" + std::to_string(getpid()) + "-" +
                      std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
    const auto fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
    if (fd < 0) {
      if (errno == EEXIST) { continue; }
      failed("Create shared VM backing");
    }
    if (shm_unlink(name.c_str()) != 0) {
      const auto reason = errno;
      static_cast<void>(close(fd));
      throw std::system_error(reason, std::generic_category(), "Unlink shared VM backing");
    }
    return fd;
  }
}
}

Backing::Backing(const std::size_t bytes) {
  static_cast<void>(page_size(bytes));
  if (!std::in_range<off_t>(bytes)) { throw std::invalid_argument("Backing extent exceeds off_t"); }
  fd_ = shared_file();
  try {
    if (ftruncate(fd_, static_cast<off_t>(bytes)) != 0) { failed("Size shared VM backing"); }
    auto *const data = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (data == MAP_FAILED) { failed("Map shared VM backing"); }
    data_ = {static_cast<std::byte *>(data), bytes};
  } catch (...) {
    static_cast<void>(close(fd_));
    throw;
  }
}

Backing::~Backing() {
  static_cast<void>(munmap(data_.data(), data_.size()));
  static_cast<void>(close(fd_));
}

AddressSpace::AddressSpace(const std::size_t bytes) : page_(page_size(bytes)) {
  auto *const data = mmap(nullptr, bytes, PROT_NONE, anonymous, -1, 0);
  if (data == MAP_FAILED) { failed("Reserve VM address space"); }
  data_ = {static_cast<std::byte *>(data), bytes};
}

AddressSpace::~AddressSpace() {
  static_cast<void>(munmap(data_.data(), data_.size()));
}

void AddressSpace::map(const Region region, const Backing *const backing) {
  if (region.size == 0 || region.size % page_ != 0 || region.address % page_ != 0 ||
      region.offset % page_ != 0 || region.address > data_.size() ||
      region.size > data_.size() - region.address ||
      (backing != nullptr && (region.offset > backing->data_.size() ||
                              region.size > backing->data_.size() - region.offset)) ||
      (backing == nullptr && region.offset != 0)) {
    throw std::invalid_argument("VM mapping exceeds its reservation or backing geometry");
  }
  const auto flags = MAP_FIXED | (backing != nullptr ? MAP_SHARED : anonymous);
  const auto protection = PROT_READ | (region.writable ? PROT_WRITE : 0);
  if (mmap(data_.data() + region.address,
           region.size,
           protection,
           flags,
           backing != nullptr ? backing->fd_ : -1,
           static_cast<off_t>(region.offset)) == MAP_FAILED) {
    failed("Map VM pages");
  }
}
}
