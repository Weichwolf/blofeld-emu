// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/clock.hpp"

#include <cstdint>
#include <dsp56300/memory.hpp>
#include <numeric>

namespace blofeld::vi::dsp {
namespace {
constexpr dsp56300::Word reset_control = 0x04401d;
constexpr dsp56300::Word enable = 0x2000;
constexpr dsp56300::Word register_mask = 0x1ff7ff;
constexpr dsp56300::Word multiplier_mask = 0xff, predivider_mask = 0x1f;
constexpr unsigned predivider_shift = 16, output_shift = 14;
}

void Clock::reset(const bool pinit) noexcept {
  // The reset fields have a valid nominal ratio for either PINIT level.
  static_cast<void>(write(reset_control | (pinit ? enable : 0)));
}

bool Clock::write(const dsp56300::Word value) noexcept {
  const auto next = value & register_mask;
  const auto low_power = std::uint32_t{1} << ((next >> 8U) & 7);
  auto ratio = Ratio{.numerator = 1, .denominator = low_power};
  if ((next & enable) != 0) {
    const auto multiplier = next & multiplier_mask;
    const auto predivider = (next >> predivider_shift) & predivider_mask;
    if (multiplier == 0 || predivider == 0) { return false; }
    const auto output = (next >> output_shift) & 3;
    // Table 5-4: OD codes 00/01/10/11 divide by 1/2/2/4; OD1 selects FM=2/4.
    const auto output_divider =
        (std::uint32_t{1} << (output & 1)) * (std::uint32_t{1} << (output >> 1U));
    const auto feedback = (output & 2) != 0 ? 4U : 2U;
    ratio = {.numerator = multiplier * feedback,
             .denominator = predivider * low_power * output_divider};
    const auto common = std::gcd(ratio.numerator, ratio.denominator);
    ratio.numerator /= common;
    ratio.denominator /= common;
  }
  control_ = next;
  ratio_ = ratio;
  return true;
}
}
