// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <span>

namespace blofeld::vi::dsp {
// DSP56371UM 3.1: invariant low RAM, with unknown contents after power-up.
// Switched banks, high RAM and data/program ROM remain separate contracts.
class Memory {
public:
  struct Write {
    dsp56300::Space space = dsp56300::Space::program;
    dsp56300::Word address = 0, value = 0;
  };

  Memory();
  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;
  [[nodiscard]] bool
  read(dsp56300::Space space, dsp56300::Word address, dsp56300::Word &value) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] std::span<const dsp56300::MemoryWindow> windows(dsp56300::Space space) noexcept;

private:
  template <std::size_t Size> struct Block {
    std::array<dsp56300::Word, Size> words{};
    std::array<std::uint8_t, Size> known{};
  };

  Block<0x1000> program_;
  static constexpr std::size_t low_x_words = 0x3000;
  Block<low_x_words> x_;
  Block<0x2000> y_;
  std::array<dsp56300::MemoryWindow, 1> x_window_, y_window_;
};
}
