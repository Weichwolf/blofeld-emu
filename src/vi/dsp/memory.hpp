// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <optional>
#include <span>

namespace blofeld::vi::dsp {
// DSP56371UM 3.1: 88K physical RAM words, retained across bank switches/reset.
class Memory {
public:
  struct Write {
    dsp56300::Space space = dsp56300::Space::program;
    dsp56300::Word address = 0, value = 0;
  };

  // Borrowed immutable ROM contents; native grants never permit stores.
  struct Rom {
    std::span<dsp56300::Word> x, y;
  };

  explicit Memory(Rom rom = {});
  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;
  void configure(dsp56300::Word omr) noexcept;
  // Publish only at an owner boundary with a new native mapping revision.
  [[nodiscard]] bool promote_initialized() noexcept;
  [[nodiscard]] bool
  read(dsp56300::Space space, dsp56300::Word address, dsp56300::Word &value) const noexcept;
  // Known reserved data writes are discarded, inferred from the original clear.
  // They never create readable RAM. Unknown I/O and program stores still fail.
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] std::span<const dsp56300::MemoryWindow> windows(dsp56300::Space space) noexcept;

private:
  template <std::size_t Size> struct Block {
    std::array<dsp56300::Word, Size> words{};
    std::array<std::uint8_t, Size> known{};
  };

  struct Cell {
    dsp56300::Word *value = nullptr;
    std::uint8_t *known = nullptr;
    bool writable = false;
  };

  [[nodiscard]] std::span<const dsp56300::MemoryWindow> view(dsp56300::Space space) const noexcept;
  [[nodiscard]] std::optional<Cell> locate(dsp56300::Space space,
                                           dsp56300::Word address) const noexcept;
  static constexpr std::size_t low_x_words = 0x3000, high_x_words = 0x6000, high_y_words = 0xa000;
  Rom rom_;
  Block<0x1000> program_;
  Block<low_x_words> x_;
  Block<0x2000> y_;
  Block<high_x_words> high_x_;
  Block<high_y_words> high_y_;
  bool initialized_x_ = false, initialized_y_ = false;
  dsp56300::Word mapping_ = dsp56300::word_mask;
  std::array<dsp56300::MemoryWindow, 3> program_windows_, x_windows_, y_windows_;
};
}
