// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_pll.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
constexpr unsigned pctl = 0xfffffd;
constexpr unsigned reset_disabled = 0x04401d;
constexpr unsigned reset_enabled = 0x04601d;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

void test_reset_pinit_is_sampled_only_at_reset() {
  blofeld::DspPll pll;
  reject([&pll] { static_cast<void>(pll.read()); }, "Unknown reset PINIT cannot become PEN=0");
  reject([&pll] { static_cast<void>(pll.target_ratio()); }, "Unknown PEN has no clock target");
  pll.observe_pinit(true);
  reject([&pll] { static_cast<void>(pll.read()); }, "PINIT changes after reset cannot resolve PEN");
  pll.reset();
  require(pll.read() == reset_enabled, "NXP section 5.2 explicitly specifies reset PCTL=04601d");
  pll.observe_pinit(false);
  require(pll.read() == reset_enabled, "PINIT is ignored after reset deassertion");
  pll.reset();
  require(pll.read() == reset_disabled, "Low PINIT clears PEN while retaining specified fields");
  pll.observe_pinit(std::nullopt);
  pll.reset();
  reject([&pll] { static_cast<void>(pll.read()); }, "An unobserved reset pin remains unknown");
  pll.write(reset_enabled);
  require(pll.read() == reset_enabled, "A full software write defines PEN without a pin guess");
}

void test_documented_programming_example_and_dividers() {
  blofeld::DspPll pll;
  constexpr std::uint64_t extal = 24576000;
  constexpr unsigned initial_control = 0x03e10b;
  constexpr unsigned final_control = 0x03a00b;
  constexpr std::uint64_t initial_hz = 45056000;
  constexpr std::uint64_t final_hz = 180224000;
  pll.write(initial_control);
  const auto first = pll.target_ratio();
  require(extal * first.numerator == initial_hz * first.denominator,
          "NXP section 5.8 step one targets 45.056 MHz from 24.576 MHz EXTAL");
  pll.write(final_control);
  const auto second = pll.target_ratio();
  require(extal * second.numerator == final_hz * second.denominator,
          "NXP step two targets 180.224 MHz; no lock or elapsed time is inferred");
  constexpr std::array<unsigned, 3> controls{0x04601d, 0x04a01d, 0x04e01d};
  constexpr std::array<unsigned, 3> denominators{4, 2, 4};
  constexpr unsigned multiplication = 29;
  for (std::size_t index = 0; index < controls.size(); ++index) {
    pll.write(pll.read() | 0x100U);
    pll.write(controls.at(index));
    const auto ratio = pll.target_ratio();
    require(ratio.numerator == multiplication && ratio.denominator == denominators.at(index),
            "OD table and feedback multiplier jointly determine the target ratio");
  }
  for (unsigned exponent = 0; exponent <= 7; ++exponent) {
    pll.write(reset_disabled | (exponent << 8U));
    const auto ratio = pll.target_ratio();
    require(ratio.numerator == 1 && ratio.denominator == (1U << exponent),
            "PLL bypass still passes through all eight low-power divider settings");
  }
  constexpr unsigned maximum_fields = 0x1fe7ff;
  constexpr unsigned maximum_denominator = 3968;
  constexpr unsigned maximum_multiplication = 255;
  pll.write(maximum_fields);
  const auto maximum = pll.target_ratio();
  require(maximum.numerator == maximum_multiplication && maximum.denominator == maximum_denominator,
          "Full-width PDF=31, MF=255 and DF=128 are decoded without plus-one encoding");
}

void test_errata_od0_change_requires_prior_division() {
  blofeld::DspPll pll;
  pll.write(reset_enabled);
  constexpr unsigned change_without_division = 0x04a01d;
  constexpr unsigned divided = 0x04611d;
  reject([&pll] { pll.write(change_without_division); },
         "DSP56371E ED57 forbids changing OD0 with the low-power divider field zero");
  require(pll.read() == reset_enabled, "Rejected ED57 sequence leaves the command unchanged");
  pll.write(divided);
  pll.write(change_without_division);
  require(pll.read() == change_without_division,
          "ED57 workaround permits changing OD0 and clearing DF in the same instruction");
}

void test_rejected_writes_and_bus_reset() {
  blofeld::DspBus bus;
  bus.write({.space = blofeld::DspSpace::x, .address = pctl, .value = reset_enabled});
  constexpr std::array<unsigned, 12> invalid{0x00601d,
                                             0x046000,
                                             0x04201d,
                                             0x04681d,
                                             0x24601d,
                                             0x44601d,
                                             0x84601d,
                                             0x104601d,
                                             0x204601d,
                                             0x404601d,
                                             0x804601d,
                                             0x8004601d};
  for (const auto value : invalid) {
    reject([&bus, value] { bus.pll().write(value); }, "Invalid PCTL command must fail visibly");
    require(bus.read({.space = blofeld::DspSpace::x, .address = pctl}) == reset_enabled,
            "Rejected PCTL writes preserve the previous command");
  }
  constexpr unsigned stop_control = reset_enabled | 0x1000;
  bus.write({.space = blofeld::DspSpace::x, .address = pctl, .value = stop_control});
  require(bus.pll().read() == stop_control, "PSTP is a retained control bit, not lock status");
  bus.pll().observe_pinit(false);
  bus.reset();
  require(bus.read({.space = blofeld::DspSpace::x, .address = pctl}) == reset_disabled,
          "External bus reset discards software PCTL and samples the observed PINIT");
  reject([&bus] { static_cast<void>(bus.read({.space = blofeld::DspSpace::y, .address = pctl})); },
         "PCTL exists only in the X I/O map");
}
}

int main() {
  try {
    test_reset_pinit_is_sampled_only_at_reset();
    test_documented_programming_example_and_dividers();
    test_errata_od0_change_requires_prior_division();
    test_rejected_writes_and_bus_reset();
    std::cout << "DSP56371 PCTL reset, PINIT sampling and documented clock targets passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
