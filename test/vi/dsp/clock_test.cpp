// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/clock.hpp"

#include <array>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Clock;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void reset_and_reserved() {
  Clock clock;
  constexpr dsp56300::Word disabled_reset = 0x04401d;
  constexpr dsp56300::Word enabled_reset = 0x04601d;
  constexpr dsp56300::Word all_bits = 0xffffff;
  constexpr dsp56300::Word implemented = 0x1ff7ff;
  clock.reset(false);
  require(clock.read() == disabled_reset &&
              clock.ratio() == Clock::Ratio{.numerator = 1, .denominator = 1},
          "PINIT low selects EXTAL directly with documented reset fields");
  constexpr Clock::Ratio reset_ratio{.numerator = 29, .denominator = 4};
  clock.reset(true);
  require(clock.read() == enabled_reset && clock.ratio() == reset_ratio,
          "PINIT high selects the documented 29/4 reset ratio");
  require(clock.write(all_bits) && clock.read() == implemented,
          "Reserved PCTL bits read zero after a full-width write");
  constexpr dsp56300::Word low_power_only = 0x700;
  require(clock.write(low_power_only) &&
              clock.ratio() == Clock::Ratio{.numerator = 1, .denominator = 128},
          "Bypassed PLL tolerates unused zero factors and still applies the low-power divider");
}

void nominal_factors() {
  Clock clock;
  constexpr dsp56300::Word reduced_output = 0x02e112;
  constexpr dsp56300::Word full_output = 0x02a012;
  require(clock.write(reduced_output) &&
              clock.ratio() == Clock::Ratio{.numerator = 9, .denominator = 2},
          "PD=2 and MF=18 are raw factors, not factor-minus-one fields");
  constexpr Clock::Ratio full_ratio{.numerator = 18, .denominator = 1};
  require(clock.write(full_output) && clock.ratio() == full_ratio,
          "Changing OD0 and DF produces the fourfold nominal frequency change");
  constexpr dsp56300::Word invalid_multiplier = full_output & ~dsp56300::Word{0xff};
  constexpr dsp56300::Word invalid_predivider = full_output & ~dsp56300::Word{0x1f0000};
  require(!clock.write(invalid_multiplier) && !clock.write(invalid_predivider) &&
              clock.read() == full_output && clock.ratio() == full_ratio,
          "An undefined enabled-PLL frequency fails without changing its previous state");
  clock.reset(false);
  constexpr dsp56300::Word stop_retention = 0x1000;
  require(clock.write(stop_retention) && clock.read() == stop_retention &&
              clock.ratio() == Clock::Ratio{.numerator = 1, .denominator = 1},
          "PSTP is retained independently of the running nominal frequency");
}

void all_dividers() {
  Clock clock;
  constexpr dsp56300::Word multiplier = 15;
  constexpr dsp56300::Word predivider = 3;
  constexpr dsp56300::Word enable = 0x2000;
  constexpr std::array<std::uint32_t, 4> output_divisors{1, 2, 2, 4};
  constexpr std::array<std::uint32_t, 4> feedback_products{30, 30, 60, 60};
  for (dsp56300::Word output = 0; output < 4; ++output) {
    for (dsp56300::Word low_power = 0; low_power < 8; ++low_power) {
      const auto value =
          enable | (predivider << 16U) | (output << 14U) | (low_power << 8U) | multiplier;
      require(clock.write(value), "All encoded divider combinations have a nominal ratio");
      const auto ratio = clock.ratio();
      const auto expected_top = feedback_products[output];
      const auto expected_bottom = predivider * output_divisors[output] * (1U << low_power);
      require(ratio.numerator * expected_bottom == ratio.denominator * expected_top,
              "Output codes 01 and 10 divide equally but select different feedback multipliers");
    }
  }
}
}

int main() try {
  reset_and_reserved();
  nominal_factors();
  all_dividers();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
