// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace blofeld {

enum class McuSpace : std::uint8_t { registers, smram, qram, reserved, flash, unmapped };

struct McuLocation {
  McuSpace space = McuSpace::unmapped;
  std::uint16_t offset = 0;
};

// Underlying storage observation; I/O and unknown bytes have no value.
struct McuMemoryByte {
  McuSpace space = McuSpace::unmapped;
  std::uint16_t offset = 0;
  std::optional<std::uint8_t> value = std::nullopt;
  bool operator==(const McuMemoryByte &) const = default;
};

// Borrowed ordinary storage selected by the committed decoder. Empty knowledge
// means initialized. A native RAM store publishes the byte and its knowledge;
// unknown reads and all read-only writes remain accesses to the owning bus.
struct McuMemoryWindow {
  std::uint16_t first = 0;
  std::span<std::uint8_t> bytes;
  std::span<std::uint8_t> known;
  bool writable = true;
};

// Normal single-chip MC9S12UF32 mapping. Power-on RAM is unknown until written.
class McuMemory {
public:
  explicit McuMemory(std::span<const std::uint8_t> flash);
  explicit McuMemory(const McuFlashImage &image);
  [[nodiscard]] McuFlashImage flash_image() const;
  [[nodiscard]] McuLocation locate(std::uint16_t address) const;
  // Sorted, disjoint RAM and known flash grants, including physical aliases.
  // Refresh after mapping retirement, flash mutation or power cycling. Ordinary
  // RAM stores update the borrowed data and knowledge; no image copy is involved.
  [[nodiscard]] std::vector<McuMemoryWindow> windows();
  // Pure inspection: no read transaction, initialization or register side effect.
  [[nodiscard]] McuMemoryByte inspect(std::uint16_t address) const;
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
  [[nodiscard]] McuMemoryWindow storage_window(McuLocation location, std::size_t size);
  friend class McuFlash;
  void commit_flash(const McuMemory &source) noexcept;
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
  std::array<std::uint8_t, smram_size> smram_initialized_{};
  std::array<std::uint8_t, qram_size> qram_initialized_{};
  std::array<std::uint8_t, map_register_count> registers_{};
  std::array<std::uint8_t, map_register_count> pending_{};
  std::bitset<map_register_count> pending_writes_;
  std::bitset<map_register_count> written_;
};

}
