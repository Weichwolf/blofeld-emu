// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
// DSP56371UM 5.6/5.7: nominal Fosc / Fextal, independent of host pacing.
class Clock {
public:
  static constexpr dsp56300::Word address = 0xfffffd;

  struct Ratio {
    std::uint32_t numerator = 1, denominator = 1;
    bool operator==(const Ratio &) const = default;
  };

  Clock() noexcept { reset(true); }

  void reset(bool pinit) noexcept;

  [[nodiscard]] dsp56300::Word read() const noexcept { return control_; }

  // Zero MF/PDF in enabled-PLL mode has no defined operating frequency.
  // Fail explicitly, preserving the previous configuration.
  [[nodiscard]] bool write(dsp56300::Word value) noexcept;

  [[nodiscard]] Ratio ratio() const noexcept { return ratio_; }

  // Nominal 60 MHz instrument time from completed base cycles. Retain sub-tick
  // phase; changing PLL factors loses less than 2^-32 of one instrument tick.
  [[nodiscard]] bool advance(std::uint64_t cycles) noexcept;

  [[nodiscard]] std::uint64_t ticks() const noexcept { return ticks_; }

  [[nodiscard]] std::optional<std::uint64_t> deadline(std::uint64_t ticks) const noexcept;

private:
  dsp56300::Word control_ = 0;
  Ratio ratio_;
  std::uint64_t cycles_ = 0, ticks_ = 0;
  std::uint32_t phase_ = 0, residue_ = 0;
};
}
