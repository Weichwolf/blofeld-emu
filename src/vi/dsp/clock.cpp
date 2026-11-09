// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/clock.hpp"

#include <cstdint>
#include <dsp56300/memory.hpp>
#include <limits>
#include <numeric>
#include <optional>

namespace blofeld::vi::dsp {
namespace {
constexpr dsp56300::Word reset_control = 0x04401d;
constexpr dsp56300::Word enable = 0x2000;
constexpr dsp56300::Word register_mask = 0x1ff7ff;
constexpr dsp56300::Word multiplier_mask = 0xff, predivider_mask = 0x1f;
constexpr unsigned predivider_shift = 16, output_shift = 14;
// 60 MHz / 11.2896 MHz, reduced before multiplying by the programmed PLL ratio.
constexpr std::uint64_t reference_top = 3125, reference_bottom = 588;
constexpr unsigned phase_bits = 32;
constexpr std::uint64_t phase_unit = std::uint64_t{1} << phase_bits;

std::optional<std::uint64_t> scaled(const std::uint64_t value,
                                    const std::uint64_t multiplier,
                                    const std::uint64_t divisor) noexcept {
  const auto whole = value / divisor;
  const auto tail = ((value % divisor) * multiplier) / divisor;
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (whole > (maximum - tail) / multiplier) { return {}; }
  return (whole * multiplier) + tail;
}
}

void Clock::reset(const bool pinit) noexcept {
  cycles_ = ticks_ = 0;
  phase_ = residue_ = 0;
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
  if (ratio != ratio_) { residue_ = 0; }
  control_ = next;
  ratio_ = ratio;
  return true;
}

bool Clock::advance(const std::uint64_t cycles) noexcept {
  if (cycles < cycles_) { return false; }
  const auto top = reference_top * ratio_.denominator;
  const auto bottom = reference_bottom * ratio_.numerator;
  const auto elapsed = cycles - cycles_;
  const auto whole = scaled(elapsed, top, bottom);
  if (!whole) { return false; }
  const auto partial = ((((elapsed % bottom) * top) % bottom) * phase_unit) + residue_;
  const auto phase = std::uint64_t{phase_} + (partial / bottom);
  const auto carry = phase >> phase_bits;
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (*whole > maximum - ticks_ || carry > maximum - ticks_ - *whole) { return false; }
  ticks_ += *whole + carry;
  phase_ = static_cast<std::uint32_t>(phase % phase_unit);
  residue_ = static_cast<std::uint32_t>(partial % bottom);
  cycles_ = cycles;
  return true;
}

std::optional<std::uint64_t> Clock::deadline(const std::uint64_t ticks) const noexcept {
  if (ticks <= ticks_) { return cycles_; }
  const auto top = reference_top * ratio_.denominator;
  const auto bottom = reference_bottom * ratio_.numerator;
  const auto elapsed = ticks - ticks_;
  const auto whole = scaled(elapsed, bottom, top);
  if (!whole) { return {}; }
  const auto partial = (((elapsed % top) * bottom) % top) * phase_unit;
  const auto phase = (std::uint64_t{phase_} * bottom) + residue_;
  const auto numerator = static_cast<std::int64_t>(partial) - static_cast<std::int64_t>(phase);
  const auto divisor = static_cast<std::int64_t>(top * phase_unit);
  const auto adjustment = (numerator / divisor) + (numerator % divisor > 0 ? 1 : 0);
  if (adjustment > 0 &&
      static_cast<std::uint64_t>(adjustment) > std::numeric_limits<std::uint64_t>::max() - *whole) {
    return {};
  }
  if (adjustment < 0 && static_cast<std::uint64_t>(-adjustment) > *whole) { return {}; }
  const auto delta = adjustment < 0 ? *whole - static_cast<std::uint64_t>(-adjustment)
                                    : *whole + static_cast<std::uint64_t>(adjustment);
  if (delta > std::numeric_limits<std::uint64_t>::max() - cycles_) { return {}; }
  return cycles_ + delta;
}
}
