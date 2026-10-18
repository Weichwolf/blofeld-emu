// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace blofeld {

enum class McuSpace : std::uint8_t { registers, smram, qram, reserved, flash, unmapped };

struct McuLocation {
  McuSpace space = McuSpace::unmapped;
  std::uint16_t offset = 0;
};

struct FlashRange {
  std::uint16_t offset = 0;
  unsigned size = 0;
};

// Normal single-chip MC9S12UF32 mapping. Power-on RAM is unknown until written.
class McuMemory {
public:
  explicit McuMemory(std::span<const std::uint8_t> flash);
  [[nodiscard]] McuLocation locate(std::uint16_t address) const;
  [[nodiscard]] std::uint8_t read(std::uint16_t address) const;
  void write(ByteWrite write);
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const;
  void check_execute(std::uint16_t address) const;
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read_register(std::uint16_t offset) const;
  void write_register(ByteWrite write);
  bool retire_instruction();
  void reset();
  void power_cycle();
  [[nodiscard]] std::uint8_t read_flash(std::uint16_t offset) const;
  void check_flash_program(std::uint16_t offset) const;
  void program_flash(MemoryWrite physical_word);
  void erase_flash(FlashRange range);
  void invalidate_flash(FlashRange range);

private:
  static constexpr std::size_t flash_size = 32768;
  static constexpr std::size_t smram_size = 3584;
  static constexpr std::size_t qram_size = 1536;
  static constexpr std::size_t map_register_count = 4;
  [[nodiscard]] bool initialized(McuLocation location) const;
  std::array<std::uint8_t, flash_size> flash_{};
  std::bitset<flash_size> flash_known_;
  std::bitset<flash_size / 2> flash_fresh_;
  std::array<std::uint8_t, smram_size> smram_{};
  std::array<std::uint8_t, qram_size> qram_{};
  std::bitset<smram_size> smram_initialized_;
  std::bitset<qram_size> qram_initialized_;
  std::array<std::uint8_t, map_register_count> registers_{};
  std::array<std::uint8_t, map_register_count> pending_{};
  std::bitset<map_register_count> pending_writes_;
  std::bitset<map_register_count> written_;
};

}
