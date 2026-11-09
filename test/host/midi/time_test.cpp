// SPDX-License-Identifier: GPL-3.0-only
#include "midi/time.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using blofeld::host::midi::Time;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <typename Error> void rejects(auto &&action, const char *message) {
  bool rejected = false;
  try {
    action();
  } catch (const Error &) { rejected = true; }
  require(rejected, message);
}
}

int main() try {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

  struct Case {
    Time time;
    std::uint32_t frequency;
    std::uint64_t ticks;
  };

  constexpr std::array cases{
      Case{.time = {}, .frequency = 60000000, .ticks = 0},
      Case{.time = {.fraction = 1, .divisor = 3}, .frequency = 60000000, .ticks = 20},
      Case{.time = {.microseconds = 166666, .fraction = 2, .divisor = 3},
           .frequency = 60000000,
           .ticks = 10000000},
      Case{.time = {.microseconds = 166666, .fraction = 2, .divisor = 3},
           .frequency = 44100,
           .ticks = 7350},
      Case{.time = {.fraction = 1, .divisor = 7}, .frequency = 60000000, .ticks = 9},
      Case{.time = {.microseconds = 1, .fraction = 2, .divisor = 7},
           .frequency = 60000000,
           .ticks = 78},
      Case{
          .time = {.microseconds = 1, .fraction = 2, .divisor = 7}, .frequency = 44100, .ticks = 1},
      Case{.time = {.microseconds = 1000000}, .frequency = 44100, .ticks = 44100},
      Case{.time = {.microseconds = maximum}, .frequency = 1, .ticks = 18446744073710ULL},
      Case{.time = {.microseconds = maximum}, .frequency = 1000000, .ticks = maximum}};
  for (const auto &test : cases) {
    require(test.time.ceil_ticks(test.frequency) == test.ticks,
            "Convert exact musical time to the first target tick at or after the event");
  }
  const Time invalid{.fraction = 1, .divisor = 1};
  rejects<std::invalid_argument>([&] { static_cast<void>(invalid.ceil_ticks(1)); },
                                 "Reject an unnormalized fractional timestamp");
  const Time undefined{.divisor = 0};
  rejects<std::invalid_argument>([&] { static_cast<void>(undefined.ceil_ticks(1)); },
                                 "Reject a zero time denominator");
  const Time zero;
  rejects<std::invalid_argument>([&] { static_cast<void>(zero.ceil_ticks(0)); },
                                 "Reject a zero destination frequency");
  constexpr std::uint32_t reference_hz = 60000000;
  const Time too_late{.microseconds = maximum};
  rejects<std::overflow_error>([&] { static_cast<void>(too_late.ceil_ticks(reference_hz)); },
                               "A timestamp must fit the destination clock");
  const Time carry{.microseconds = maximum, .fraction = 1, .divisor = 2};
  constexpr std::uint32_t microsecond_clock = 1000000;
  rejects<std::overflow_error>([&] { static_cast<void>(carry.ceil_ticks(microsecond_clock)); },
                               "Ceiling cannot wrap a maximum destination timestamp");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
