// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstddef>
#include <span>

namespace blofeld::host::memory {
class Backing {
public:
  explicit Backing(std::size_t bytes);
  ~Backing();
  Backing(const Backing &) = delete;
  Backing &operator=(const Backing &) = delete;

  [[nodiscard]] std::span<std::byte> bytes() const noexcept { return data_; }

private:
  friend class AddressSpace;
  int fd_ = -1;
  std::span<std::byte> data_;
};

// Reserve first, then map declared pages. Unmapped pages remain inaccessible.
class AddressSpace {
public:
  struct Region {
    std::size_t address = 0, offset = 0, size = 0;
    bool writable = false;
  };

  explicit AddressSpace(std::size_t bytes);
  ~AddressSpace();
  AddressSpace(const AddressSpace &) = delete;
  AddressSpace &operator=(const AddressSpace &) = delete;

  [[nodiscard]] std::span<std::byte> bytes() const noexcept { return data_; }

  // A null backing supplies private zero pages, including writable scratch.
  void map(Region region, const Backing *backing = nullptr);

private:
  std::size_t page_;
  std::span<std::byte> data_;
};
}
