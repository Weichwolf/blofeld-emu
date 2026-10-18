// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {

struct DspClockRatio {
  std::uint32_t numerator = 1;
  std::uint32_t denominator = 1;
};

// DSP56371 PCTL command state. A target ratio is not an observed, locked clock.
class DspPll {
public:
  DspPll();
  void observe_pinit(std::optional<bool> level);
  void reset();
  [[nodiscard]] std::uint32_t read() const;
  void write(std::uint32_t value);
  [[nodiscard]] DspClockRatio target_ratio() const;

private:
  std::optional<bool> pinit_;
  std::uint32_t control_ = 0;
  bool pen_known_ = false;
};

}
