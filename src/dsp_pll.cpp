// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_pll.hpp"

#include "blofeld/bus.hpp"

#include <cstdint>
#include <numeric>
#include <optional>

namespace blofeld {
namespace {
constexpr unsigned reset_control = 0x04401d;
constexpr unsigned allowed = 0x1ff7ff;
constexpr unsigned pen = 0x2000;
constexpr unsigned od0 = 0x4000;
constexpr unsigned od1 = 0x8000;
constexpr unsigned output_dividers = od0 | od1;
constexpr unsigned multiplication_mask = 0xff;
constexpr unsigned predivider_mask = 0x1f;
constexpr unsigned division_mask = 0x700;
}

DspPll::DspPll() {
  reset();
}

void DspPll::observe_pinit(const std::optional<bool> level) {
  pinit_ = level;
}

void DspPll::reset() {
  control_ = reset_control | (pinit_.value_or(false) ? pen : 0U);
  pen_known_ = pinit_.has_value();
}

std::uint32_t DspPll::read() const {
  if (!pen_known_) {
    throw Fault("Unknown DSP PCTL read at X:0xfffffd: PINIT was not observed during reset");
  }
  return control_;
}

void DspPll::write(const std::uint32_t value) {
  if ((value & ~allowed) != 0) {
    throw Fault("Unsupported DSP PCTL reserved-bit write at X:0xfffffd value=" + hex(value, 6));
  }
  if (((value >> 16U) & predivider_mask) == 0 || (value & multiplication_mask) == 0) {
    throw Fault("Invalid DSP PCTL zero PDF/MF at X:0xfffffd value=" + hex(value, 6));
  }
  if ((value & pen) != 0 && (value & output_dividers) == 0) {
    throw Fault("Unqualified DSP PCTL PLL divide-by-one at X:0xfffffd value=" + hex(value, 6));
  }
  if (((control_ ^ value) & od0) != 0 && (control_ & division_mask) == 0) {
    throw Fault("Unqualified DSP PCTL OD0 change while DF=0 (DSP56371E ED57) at X:0xfffffd "
                "value=" +
                hex(value, 6));
  }
  control_ = value;
  pen_known_ = true;
}

DspClockRatio DspPll::target_ratio() const {
  const auto control = read();
  const auto division = 1U << ((control >> 8U) & 7U);
  if ((control & pen) == 0) { return {.denominator = division}; }
  const auto predivision = (control >> 16U) & predivider_mask;
  const auto multiplication = control & multiplication_mask;
  const unsigned feedback = (control & od1) != 0 ? 4 : 2;
  const unsigned output = ((control & od0) != 0 ? 2U : 1U) * (feedback / 2);
  const auto numerator = multiplication * feedback;
  const auto denominator = predivision * division * output;
  const auto common = std::gcd(numerator, denominator);
  return {.numerator = numerator / common, .denominator = denominator / common};
}

}
