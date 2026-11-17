// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
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

struct Import {
  std::string_view name, abi;
  const void *address = nullptr;
};

class Code {
public:
  struct TextRange {
    std::uintptr_t address = 0;
    std::uint64_t size = 0;
  };

  struct Site {
    std::uintptr_t address = 0;
    std::uint64_t size = 0, variant = 0;
    std::uint32_t guest_pc = 0;
  };

  struct FaultMap {
    std::span<const TextRange> text_ranges;
    std::span<const Site> sites;
  };

  explicit Code(std::span<const Module> modules,
                std::span<const Import> imports = {},
                const std::filesystem::path &cache = {});
  ~Code();
  Code(const Code &) = delete;
  Code &operator=(const Code &) = delete;
  [[nodiscard]] const void *address(std::string_view name) const;
  [[nodiscard]] std::uint64_t native_bytes() const noexcept;
  // Immutable linked text and guest line intervals; owned for this Code lifetime.
  [[nodiscard]] FaultMap fault_map() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
