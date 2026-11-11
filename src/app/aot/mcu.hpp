// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "aot/code.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <string>

namespace blofeld::app::aot {
class McuBinding {
public:
  McuBinding();
  McuBinding(const McuBinding &) = delete;
  McuBinding &operator=(const McuBinding &) = delete;
  [[nodiscard]] static std::span<const std::byte> bitcode() noexcept;

  [[nodiscard]] std::span<const host::aot::Import> imports() const noexcept { return imports_; }

private:
  static constexpr std::size_t channel_count = 18;
  std::array<std::string, channel_count> signatures_;
  std::array<host::aot::Import, channel_count> imports_;
};
}
