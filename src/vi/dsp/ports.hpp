// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <dsp56300/memory.hpp>

namespace blofeld::vi::dsp {
enum class Port : std::uint8_t { c, d, e, f };

class Ports {
public:
  struct Write {
    dsp56300::Space space = dsp56300::Space::x;
    dsp56300::Word address = 0, value = 0;
  };

  struct Input {
    dsp56300::Word mask = 0, known = 0, value = 0;
  };

  struct Output {
    dsp56300::Word mask = 0, known = 0, value = 0;
  };

  Ports();
  void reset() noexcept;
  [[nodiscard]] static bool contains(dsp56300::Space space, dsp56300::Word address) noexcept;
  [[nodiscard]] bool
  read(dsp56300::Space space, dsp56300::Word address, dsp56300::Word &value) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  void input(Port port, Input sample);
  [[nodiscard]] Output output(Port port) const;
  [[nodiscard]] dsp56300::Word peripheral(Port port) const;

private:
  struct Registers {
    dsp56300::Word data = 0, known = 0, direction = 0, function = 0;
    Input input;
  };

  std::array<Registers, 4> registers_;
};
}
