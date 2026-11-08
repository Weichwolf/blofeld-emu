// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/flash_image.hpp"

#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld::vi::mcu {
enum class Space : std::uint8_t { unmapped, registers, system_ram, queue_ram, flash, reserved };

struct Location {
  Space space = Space::unmapped;
  std::uint16_t offset = 0;
  bool operator==(const Location &) const = default;
};

// Normal single-chip UF32 map with SMRAM in its reset (3.5 KiB) mode.
// Owns volatile RAM; borrows persistent flash. One MCU execution owner.
class Memory {
public:
  explicit Memory(FlashImage &flash);
  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;
  void reset();
  void power_on();
  [[nodiscard]] Location locate(std::uint16_t address) const noexcept;
  // Sorted direct grants. Keep the descriptors until Core construction/rebind;
  // backing arrays and this object must outlive Core. Rebuild only for a real
  // map/code/knowledge transition, never for ordinary RAM accesses.
  [[nodiscard]] std::vector<cpu12::Ram> regions();
  [[nodiscard]] bool read(std::uint16_t address, std::uint8_t &value) const noexcept;
  [[nodiscard]] bool write(cpu12::ByteWrite access) noexcept;
  [[nodiscard]] std::uint8_t read_mapping(std::uint16_t offset) const;
  // Stage known mapping-register writes; true requests an instruction-boundary
  // return. Commit and rebind after that complete instruction, not every cycle.
  bool write_mapping(cpu12::ByteWrite access);
  bool commit_mapping() noexcept;

private:
  [[nodiscard]] cpu12::Ram region(Location start, std::size_t size);
  FlashImage &flash_;
  static constexpr std::size_t system_bytes = 3584;
  static constexpr std::size_t queue_bytes = 1536;
  std::array<std::uint8_t, system_bytes> system_{};
  std::array<std::uint8_t, system_bytes> system_known_{};
  std::array<std::uint8_t, queue_bytes> queue_{};
  std::array<std::uint8_t, queue_bytes> queue_known_{};
  std::array<std::uint8_t, 4> mapping_{};
  std::array<std::uint8_t, 4> pending_{};
  std::uint8_t written_ = 0;
};
}
