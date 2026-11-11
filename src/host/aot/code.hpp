// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

namespace blofeld::host::aot {
struct Export {
  std::string_view name, abi;
};

struct Module {
  std::span<const std::byte> bitcode;
  std::span<const Export> exports;
};

class Code {
public:
  explicit Code(std::span<const Module> modules);
  ~Code();
  Code(const Code &) = delete;
  Code &operator=(const Code &) = delete;
  [[nodiscard]] const void *address(std::string_view name) const;
  [[nodiscard]] std::uint64_t native_bytes() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
