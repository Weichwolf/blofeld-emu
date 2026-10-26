// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_icg_clocks.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using Mode = Hcs08IcgClockMode;
using Domain = Hcs08IcgClockDomain;
using Phase = Hcs08IcgFeePhase;
using Frequency = Hcs08IcgFrequency;
using Ratio = Hcs08IcgTargetRatio;
using Point = Hcs08IcgClockPoint;
constexpr auto contract = Hcs08IcgClockContract::candidate_gt8a_documented_routes;
constexpr std::array<unsigned, 8> multipliers{4, 6, 8, 10, 12, 14, 16, 18};
constexpr std::array<unsigned, 8> reductions{1, 2, 4, 8, 16, 32, 64, 128};
constexpr std::array domains{Domain::icgout, Domain::bus, Domain::fixed};
constexpr unsigned all_bytes = 256, mode_step = 8, multiplier_step = 16, range_step = 64;
constexpr unsigned internal_prescale = 64, internal_divider = 7;
constexpr unsigned self_hz = 16000000, transient_hz = 12000000;
constexpr unsigned high_hz = 2000000, low_hz = 32768;
constexpr unsigned clock_one = 0x28, clock_two = 0x30;
constexpr unsigned fee_high_control = 0x78, below_threshold_control = 0x11;
constexpr unsigned one_mhz = 1000000, three_mhz = 3000000;
constexpr unsigned nominal_bus_numerator = 77760000, source_pc = 0xe1b0, source_ccr = 0x69;
unsigned target_cases = 0, route_cases = 0, controls = 0;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

Frequency frequency(const std::uint64_t numerator, const std::uint64_t denominator = 1) {
  const auto common = std::gcd(numerator, denominator);
  return {.numerator = numerator / common, .denominator = denominator / common};
}

void check(Hcs08IcgClocks &clocks,
           const Point &point,
           const Domain domain,
           const std::optional<Frequency> expected) {
  require(clocks.clock(domain, point) == expected,
          "Independent literal source/divider/XCLK expectation");
  ++route_cases;
}

void targets() {
  for (unsigned first = 0; first < all_bytes; ++first) {
    for (unsigned second = 0; second < all_bytes; ++second) {
      const unsigned mode = (first / mode_step) % 4;
      std::optional<Ratio> expected;
      if (mode == 1 || mode == 3) {
        const unsigned prescale =
            mode == 1 || (first / range_step) % 2 == 0 ? internal_prescale : 1U;
        const unsigned numerator = prescale * multipliers[(second / multiplier_step) % 8];
        const unsigned denominator = reductions[second % 8] * (mode == 1 ? internal_divider : 1U);
        const unsigned common = std::gcd(numerator, denominator);
        expected = Ratio{.numerator = numerator / common, .denominator = denominator / common};
      }
      require(Hcs08IcgClocks::target_ratio(static_cast<std::uint8_t>(first),
                                           static_cast<std::uint8_t>(second)) == expected,
              "All control words use independent literal N/R table and mode-dependent prescale");
      ++target_cases;
    }
  }
}

void checked_row(Hcs08IcgClocks &clocks,
                 const Point &point,
                 const Frequency out,
                 const std::optional<Frequency> fixed) {
  check(clocks, point, Domain::icgout, out);
  check(clocks, point, Domain::bus, frequency(out.numerator, out.denominator * 2));
  check(clocks, point, Domain::fixed, fixed);
}

void route_word(Hcs08IcgClocks &clocks, const unsigned first, const unsigned second) {
  const std::uint64_t reduce = reductions[second % 8];
  const unsigned multiply = multipliers[(second / multiplier_step) % 8];
  const bool high = (first / range_step) % 2 != 0;
  const std::uint64_t prescale = high ? 1U : internal_prescale;
  const std::uint64_t external = high ? high_hz : low_hz;
  const std::uint64_t settled = external * prescale * multiply;
  Point point{.mode = Mode::off,
              .control1 = static_cast<std::uint8_t>(first),
              .control2 = static_cast<std::uint8_t>(second)};
  for (const auto domain : domains) { check(clocks, point, domain, {}); }
  point.mode = Mode::scm;
  point.dco = Frequency{.numerator = self_hz};
  checked_row(clocks, point, frequency(self_hz, reduce), {});
  point.mode = Mode::fei;
  point.dco = Frequency{.numerator = transient_hz};
  checked_row(clocks, point, frequency(transient_hz, reduce), {});
  point.mode = Mode::fbe;
  point.dco.reset();
  point.external = Frequency{.numerator = external};
  checked_row(clocks, point, frequency(external, reduce), frequency(external, 2 * reduce));
  point.mode = Mode::fee;
  point.dco = Frequency{.numerator = settled};
  point.locked = true;
  point.fee_phase = Phase::after_first_lock;
  const auto fixed =
      prescale * multiply >= 4 * reduce ? frequency(external, 2) : frequency(settled, 2 * reduce);
  checked_row(clocks, point, frequency(settled, reduce), fixed);
  point.dco = Frequency{.numerator = transient_hz};
  point.locked = false;
  checked_row(clocks, point, frequency(transient_hz, reduce), frequency(transient_hz, 2 * reduce));
  point.fee_phase = Phase::initial_or_multiplier_relock;
  checked_row(
      clocks, point, frequency(transient_hz, 2 * reduce), frequency(transient_hz, 4 * reduce));
}

void routes(Hcs08IcgClocks &clocks) {
  for (unsigned first = 0; first < all_bytes; ++first) {
    for (unsigned second = 0; second < all_bytes; ++second) { route_word(clocks, first, second); }
  }
  constexpr unsigned expected_routes = 1376256;
  require(route_cases == expected_routes,
          "All seven actual-mode/history rows and three consumer domains ran");
}

void literal_examples(Hcs08IcgClocks &clocks) {
  struct Target {
    unsigned first, second, numerator, denominator;
  };

  constexpr std::array<Target, 6> examples{
      {{.first = clock_one, .second = clock_two, .numerator = 640, .denominator = 7},
       {.first = 0x68, .second = 0xb0, .numerator = 640, .denominator = 7},
       {.first = clock_one, .second = 0x31, .numerator = 320, .denominator = 7},
       {.first = 0x38, .second = 0, .numerator = 256, .denominator = 1},
       {.first = 0x78, .second = clock_two, .numerator = 10, .denominator = 1},
       {.first = 0x78, .second = 0x37, .numerator = 5, .denominator = 64}}};
  for (const auto example : examples) {
    require(Hcs08IcgClocks::target_ratio(static_cast<std::uint8_t>(example.first),
                                         static_cast<std::uint8_t>(example.second)) ==
                Ratio{.numerator = example.numerator, .denominator = example.denominator},
            "Pinned source-request and primary example ratios are exact fractions");
  }
  Point point{.mode = Mode::fei,
              .control1 = clock_one,
              .control2 = clock_two,
              .dco = Frequency{.numerator = transient_hz}};
  checked_row(clocks, point, Frequency{.numerator = transient_hz}, {});
  point.mode = Mode::scm;
  constexpr unsigned reset_dco = 8000000;
  point.dco = Frequency{.numerator = reset_dco};
  checked_row(clocks, point, Frequency{.numerator = reset_dco}, {});
  point.mode = Mode::fei;
  constexpr unsigned nominal_numerator = 155520000;
  point.dco = Frequency{.numerator = nominal_numerator, .denominator = 7};
  check(
      clocks, point, Domain::bus, Frequency{.numerator = nominal_bus_numerator, .denominator = 7});
  point.mode = Mode::fee;
  point.control1 = fee_high_control;
  point.dco = Frequency{.numerator = transient_hz};
  point.external = Frequency{.numerator = high_hz};
  point.locked = false;
  point.fee_phase = Phase::initial_or_multiplier_relock;
  checked_row(clocks, point, frequency(transient_hz, 2), frequency(transient_hz, 4));
  point.fee_phase = Phase::after_first_lock;
  checked_row(clocks, point, Frequency{.numerator = transient_hz}, frequency(transient_hz, 2));
  constexpr unsigned matched_dco = 20000000;
  point.dco = Frequency{.numerator = matched_dco};
  point.locked = true;
  point.fee_phase = Phase::unqualified;
  check(clocks, point, Domain::fixed, Frequency{.numerator = one_mhz});
}

void boundaries(Hcs08IcgClocks &clocks) {
  struct Boundary {
    unsigned first, second, dco, external, fixed;
  };

  constexpr std::array<Boundary, 6> cases{
      {{.first = 0x78, .second = 0, .dco = 8000000, .external = high_hz, .fixed = 1000000},
       {.first = 0x78, .second = 0x11, .dco = 12000000, .external = high_hz, .fixed = 3000000},
       {.first = 0x78, .second = 0x21, .dco = 16000000, .external = high_hz, .fixed = 1000000},
       {.first = 0x78, .second = 0x31, .dco = 20000000, .external = high_hz, .fixed = 1000000},
       {.first = 0x38, .second = 7, .dco = 8388608, .external = low_hz, .fixed = 32768},
       {.first = 0x38, .second = 0x27, .dco = 16777216, .external = low_hz, .fixed = 16384}}};
  for (const auto sample : cases) {
    const Point point{.mode = Mode::fee,
                      .control1 = static_cast<std::uint8_t>(sample.first),
                      .control2 = static_cast<std::uint8_t>(sample.second),
                      .dco = Frequency{.numerator = sample.dco},
                      .external = Frequency{.numerator = sample.external},
                      .locked = true,
                      .fee_phase = Phase::after_first_lock};
    check(clocks, point, Domain::fixed, Frequency{.numerator = sample.fixed});
  }
  Point point{.mode = Mode::fee,
              .control1 = fee_high_control,
              .control2 = below_threshold_control,
              .dco = Frequency{.numerator = transient_hz},
              .fee_phase = Phase::after_first_lock};
  check(clocks, point, Domain::fixed, Frequency{.numerator = three_mhz});
  constexpr auto largest = std::numeric_limits<std::uint64_t>::max();
  point.mode = Mode::fbe;
  point.control2 = 7;
  point.external = Frequency{.numerator = largest, .denominator = largest};
  check(clocks, point, Domain::icgout, Frequency{.numerator = 1, .denominator = 128});
  point.external = Frequency{.numerator = largest};
  check(clocks, point, Domain::bus, Frequency{.numerator = largest, .denominator = 256});
  point.control2 = 1;
  point.external = Frequency{.numerator = 4, .denominator = 3};
  check(clocks, point, Domain::bus, Frequency{.numerator = 1, .denominator = 3});
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected strict clock refusal");
}

void refusal(Hcs08IcgClocks &clocks,
             const Point &point,
             const Domain domain,
             const std::string_view why) {
  const auto before = point;
  const auto text = rejected([&] { static_cast<void>(clocks.clock(domain, point)); });
  require(text.contains(why) && text.contains("address=0x004a") && text.contains("pc=0xe1b0") &&
              text.contains("ccr=0x0069") && text.contains("agent=CPU") && point == before,
          "Unknown/unsupported actual-clock facts fail with unchanged inputs and complete context");
  ++controls;
}

void strict(Hcs08IcgClocks &clocks) {
  for (const auto value :
       {Hcs08IcgClockContract::unqualified, Hcs08IcgClockContract::unsupported}) {
    require(rejected([&] { const Hcs08IcgClocks other(value); }).contains("configure"),
            "Explicit candidate contract required");
    ++controls;
  }
  Point point{.mode = Mode::scm, .dco = Frequency{.numerator = transient_hz}};
  for (const auto domain : {Domain::unqualified, Domain::unsupported}) {
    refusal(clocks, point, domain, "domain");
  }
  for (const auto mode : {Mode::unqualified, Mode::unsupported}) {
    point.mode = mode;
    refusal(clocks, point, Domain::bus, "mode");
  }
  point.dco.reset();
  for (const auto mode : {Mode::scm, Mode::fei, Mode::fbe}) {
    point.mode = mode;
    refusal(clocks, point, Domain::bus, "observed");
  }
  point.mode = Mode::scm;
  for (const auto rate : {Frequency{.numerator = 0}, Frequency{.numerator = 1, .denominator = 0}}) {
    point.dco = rate;
    refusal(clocks, point, Domain::bus, "positive");
  }
  point = {.mode = Mode::fee,
           .control1 = fee_high_control,
           .control2 = 0,
           .dco = Frequency{.numerator = transient_hz}};
  refusal(clocks, point, Domain::bus, "history");
  point.fee_phase = Phase::unsupported;
  refusal(clocks, point, Domain::bus, "phase");
  point.fee_phase = Phase::after_first_lock;
  refusal(clocks, point, Domain::fixed, "LOCK");
  point.fee_phase = Phase::initial_or_multiplier_relock;
  point.locked = true;
  refusal(clocks, point, Domain::bus, "conflicts");
  for (const auto mode : {Mode::off, Mode::scm, Mode::fbe}) {
    point.mode = mode;
    refusal(clocks, point, Domain::fixed, "LOCK");
  }
  point = {.mode = Mode::fbe,
           .control2 = 1,
           .external =
               Frequency{.numerator = 1, .denominator = std::numeric_limits<std::uint64_t>::max()}};
  refusal(clocks, point, Domain::icgout, "overflow");
  constexpr unsigned expected_controls = 19;
  require(controls == expected_controls, "Every directed strict refusal ran");
}
}

int main() {
  try {
    Hcs08IcgClocks clocks(contract);
    clocks.context({.pc = source_pc, .ccr = source_ccr});
    targets();
    routes(clocks);
    literal_examples(clocks);
    boundaries(clocks);
    strict(clocks);
    std::cout << "ICG clock routes: " << target_cases << " target words, " << route_cases
              << " actual-route/literal checks, " << controls
              << " strict controls. No clock/status generation.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
