// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/clock.hpp"

#include <array>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Clock;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void fractional_boundaries() {
  Clock clock;
  constexpr std::uint64_t period = 4263;
  constexpr std::uint64_t ticks = 3125;
  for (const auto cycles : {std::uint64_t{0}, std::uint64_t{1}, period - 1, period, period + 1}) {
    require(clock.advance(cycles) && clock.fractional() == (cycles % period != 0) &&
                clock.ticks() == (cycles * ticks) / period,
            "Exact 3125/4263 phase distinguishes a future position from the same integer tick");
  }
  clock.reset(true);
  require(!clock.fractional(), "A new source epoch cannot retain an older fractional position");
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
  constexpr Clock::Ratio maximum_fields{.numerator = 255, .denominator = 3968};
  require(clock.write(all_bits) && clock.read() == implemented && clock.ratio() == maximum_fields,
          "Reserved bits read zero while maximum valid PLL fields retain their reduced ratio");
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

void instrument_time() {
  constexpr std::array<dsp56300::Word, 4> modes{0, 0x04601d, 0x02a012, 0x700};
  constexpr std::uint64_t instrument_hz = 60000000;
  constexpr std::uint64_t oscillator_hz = 11289600;
  constexpr std::uint64_t last_cycle = 300;
  constexpr std::uint64_t future_ticks = 1000;
  for (const auto mode : modes) {
    Clock clock;
    require(clock.write(mode), "Select a nominal reference, PLL or low-power clock");
    const auto ratio = clock.ratio();
    const auto top = instrument_hz * ratio.denominator;
    const auto bottom = oscillator_hz * ratio.numerator;
    for (std::uint64_t cycles = 0; cycles <= last_cycle; ++cycles) {
      require(clock.advance(cycles) && clock.ticks() == (cycles * top) / bottom,
              "Repeated small advances retain the independently calculated fractional phase");
      require(clock.advance(cycles) && clock.ticks() == (cycles * top) / bottom,
              "Advancing again at the same cycle retains the original tick");
      const auto target = clock.ticks() + future_ticks;
      const auto expected = ((target * bottom) + top - 1) / top;
      require(clock.deadline(target) == expected,
              "An instrument timestamp selects the first complete base-cycle boundary");
    }
    require(!clock.advance(0), "A core clock cannot move backwards");
  }
}

void large_advances_and_deadlines() {
  Clock clock;
  constexpr dsp56300::Word full_output = 0x02a012;
  constexpr std::uint64_t top = 3125;
  constexpr std::uint64_t bottom = 10584;
  constexpr std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
  constexpr unsigned phase_bits = 32;
  constexpr auto short_deadline = (maximum >> phase_bits) / bottom;
  require(clock.write(full_output), "Select the native firmware run clock");
  for (const auto target : {short_deadline - 1, short_deadline, short_deadline + 1}) {
    const auto first_cycle = ((target * bottom) + top - 1) / top;
    require(clock.deadline(target) == first_cycle,
            "The rounding of a representable deadline is independent of its magnitude");
  }
  constexpr auto tail = (maximum % bottom) * top;
  constexpr auto expected = ((maximum / bottom) * top) + (tail / bottom);
  require(clock.advance(maximum) && clock.ticks() == expected &&
              clock.fractional() == (tail % bottom != 0) && clock.deadline(expected) == maximum &&
              !clock.deadline(expected + 1),
          "The last base cycle remains representable without overflowing reference time");
}

void instrument_clock_changes() {
  Clock clock;
  constexpr dsp56300::Word full_output = 0x02a012;
  require(clock.advance(1) && clock.ticks() == 0 && clock.write(full_output) && clock.advance(2) &&
              clock.ticks() == 1 && clock.deadline(2) == 6,
          "A PLL change preserves the elapsed fractional reference time");
  const auto previous = clock.ticks();
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  require(!clock.deadline(maximum) && clock.ticks() == previous,
          "An unrepresentable native deadline fails without changing time");
  constexpr dsp56300::Word low_power = 0x700;
  require(clock.write(low_power) && !clock.advance(maximum) && clock.ticks() == previous,
          "Elapsed-time overflow cannot partially commit a new clock state");
  clock.reset(true);
  require(clock.ticks() == 0 && clock.advance(0), "Reset creates a fresh local clock epoch");
}
}

int main() try {
  fractional_boundaries();
  reset_and_reserved();
  nominal_factors();
  all_dividers();
  instrument_time();
  large_advances_and_deadlines();
  instrument_clock_changes();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
