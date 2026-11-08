// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::vi {
// BLFNAND v1 storage. Keep the encoded backing and an O(1) page index;
// no cell simulation, per-page allocations or second payload copy.
class MediaImage {
public:
  static constexpr std::uint32_t page_bytes = 528, pages = 131072, block_pages = 32;
  static constexpr std::size_t record_bytes = 8 + (page_bytes * 2);
  static constexpr std::size_t max_bytes =
      36 + ((pages / block_pages) * 4) + (std::size_t{pages} * record_bytes);

  struct Read {
    std::uint32_t page = 0, column = 0;
  };

  MediaImage();
  explicit MediaImage(std::vector<std::uint8_t> bytes);
  [[nodiscard]] std::span<const std::uint8_t> page(std::uint32_t index) const noexcept;
  [[nodiscard]] bool read(Read access, std::uint8_t &value) const noexcept;

  // false is a known device failure; unknown metadata cannot claim an outcome.
  [[nodiscard]] std::optional<bool> erase(std::uint32_t block);
  void interrupt_erase(std::uint32_t block) noexcept;
  // Refresh the checksum only on explicit serialization. Views are borrowed;
  // materializing missing pages may invalidate previous page/encoded views.
  [[nodiscard]] std::span<const std::uint8_t> encoded() noexcept;

private:
  void validate();
  std::vector<std::uint8_t> bytes_;
  std::array<std::uint32_t, pages> offsets_;
  std::array<std::uint32_t, pages / block_pages> block_offsets_;
  bool checksum_dirty_ = false;
};
}
