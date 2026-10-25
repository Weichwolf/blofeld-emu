// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld::usbip {
enum class Field : std::uint8_t {
  command = 0,
  sequence = 4,
  device = 8,
  direction = 12,
  endpoint = 16,
  flags_status_target = 20,
  length = 24,
  frame = 28,
  packets = 32,
  interval_errors = 36,
  setup = 40
};
inline constexpr unsigned submit = 1;
inline constexpr unsigned unlink = 2;
inline constexpr unsigned return_submit = 3;
inline constexpr unsigned return_unlink = 4;
[[nodiscard]] std::uint32_t word(std::span<const std::uint8_t> bytes, Field field);
void put(std::span<std::uint8_t> bytes, Field field, std::uint32_t value);
}
