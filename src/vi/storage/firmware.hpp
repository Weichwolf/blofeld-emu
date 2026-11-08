// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::vi {
// Original 1.25 update transport, not an execution image or device identity.
// Filesystem/archive handling belongs to the app and host. Device loading must
// preserve these bytes, including complete words beyond the declared main block.
struct FirmwareUpdate {
  static constexpr std::size_t flash_bytes = 32768;
  static constexpr std::size_t block_count = 6;
  std::array<std::uint8_t, flash_bytes> mcu_flash{};
  std::vector<std::uint8_t> nand_program;
  std::array<std::vector<std::uint8_t>, block_count> blocks;

  [[nodiscard]] static FirmwareUpdate from_midi(std::span<const std::uint8_t> midi);
};
}
