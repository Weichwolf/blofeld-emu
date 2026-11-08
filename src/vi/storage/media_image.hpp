// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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

  [[nodiscard]] std::span<const std::uint8_t> encoded() const noexcept { return bytes_; }

private:
  void validate();
  std::vector<std::uint8_t> bytes_;
  std::array<std::uint32_t, pages> offsets_;
};
}
