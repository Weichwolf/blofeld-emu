// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <dsp56300/memory.hpp>

namespace blofeld::vi::dsp {
// DSP56371UM 4.4: stored priorities and external IRQ trigger modes. Delivery
// is separate and requires an actual device request; configuration creates none.
class Interrupts {
public:
  struct Write {
    dsp56300::Word address = 0, value = 0;
  };

  static constexpr dsp56300::Word core_address = 0xffffff;
  static constexpr dsp56300::Word peripheral_address = 0xfffffe;

  [[nodiscard]] static bool contains(const dsp56300::Word address) noexcept {
    return address == core_address || address == peripheral_address;
  }

  void reset() noexcept { core_ = peripheral_ = 0; }

  [[nodiscard]] bool read(const dsp56300::Word address, dsp56300::Word &value) const noexcept {
    if (!contains(address)) { return false; }
    value = address == core_address ? core_ : peripheral_;
    return true;
  }

  [[nodiscard]] bool write(const Write access) noexcept {
    if (!contains(access.address) || access.value > dsp56300::word_mask) { return false; }
    constexpr dsp56300::Word peripheral_mask = 0x030fcf;
    if (access.address == core_address) {
      core_ = access.value;
    } else {
      peripheral_ = access.value & peripheral_mask;
    }
    return true;
  }

private:
  dsp56300::Word core_ = 0, peripheral_ = 0;
};
}
