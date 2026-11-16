// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/memory.hpp"

#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::vi::mcu {
class Program {
public:
  Program(cpu12::NativeImage image, cpu12::CodeImage source);
  Program(const Program &) = delete;
  Program &operator=(const Program &) = delete;
  Program(Program &&) noexcept = default;
  Program &operator=(Program &&) noexcept = default;

  [[nodiscard]] bool contains(std::span<const std::uint64_t> mounted, cpu12::Address pc) const;
  [[nodiscard]] std::vector<std::uint64_t> admit(Memory &memory, cpu12::Address pc) const;
  void install(cpu12::Core &core, std::span<const std::uint64_t> mounted) const;

  [[nodiscard]] cpu12::PreparationReport report() const noexcept { return image_.report(); }

private:
  struct Source {
    explicit Source(const cpu12::CodeVariant &variant);
    [[nodiscard]] bool contains(cpu12::Address pc) const;
    [[nodiscard]] bool matches(const Memory &memory) const;
    std::uint64_t id;
    std::vector<cpu12::Byte> bytes;
    std::vector<cpu12::CodeSpan> code;
  };

  cpu12::NativeImage image_;
  std::vector<Source> sources_;
};
}
