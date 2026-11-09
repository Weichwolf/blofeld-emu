// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <dsp56300/memory.hpp>
#include <optional>
#include <span>

namespace blofeld::vi::dsp {
// DSP56371UM 4.4: configured priorities and arbitration of actual device requests.
class Interrupts {
public:
  struct Write {
    dsp56300::Word address = 0, value = 0;
  };

  struct Request {
    dsp56300::Word vector = 0; // Relative to VBA.
    unsigned level = 0;
    bool operator==(const Request &) const = default;
  };

  [[nodiscard]] std::optional<Request> select(std::span<const dsp56300::Word> vectors,
                                              dsp56300::Word status) const noexcept;

  static constexpr dsp56300::Word core_address = 0xffffff;
  static constexpr dsp56300::Word peripheral_address = 0xfffffe;

  [[nodiscard]] static bool contains(const dsp56300::Word address) noexcept {
    return address == core_address || address == peripheral_address;
  }

  [[nodiscard]] bool external_edge(const dsp56300::Word vector) const noexcept {
    constexpr dsp56300::Word first = 0x10;
    constexpr dsp56300::Word last = 0x16;
    return vector >= first && vector <= last && (vector & 1) == 0 &&
           ((core_ >> (((vector - first) / 2) * 3)) & 4) != 0;
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
  [[nodiscard]] unsigned priority(dsp56300::Word vector) const noexcept;
  dsp56300::Word core_ = 0, peripheral_ = 0;
};
}
