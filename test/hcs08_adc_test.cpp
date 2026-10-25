// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_adc.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using test::hcs08::rejected;
using test::hcs08::require;
namespace adc = hcs08_adc;
constexpr auto contract = Hcs08AdcContract::candidate_gt8a_reference_stable_input;
constexpr Hcs08AdcReference reference{
    .bus_hz = 2000000, .supply_uv = 3300000, .low_uv = 0, .high_uv = 3072000};
constexpr unsigned single_periods = 60, continuous_periods = 56, midpoint_uv = 1539000;
constexpr unsigned unsigned_right = 0xc0, ready_irq = 0xc0, active_irq = 0x40;
constexpr unsigned byte_mask = 255, maximum_bus = 20000000, pc = 0x1234, flags = 0x68;
constexpr unsigned status_mask = 0x7f, reference_high_channel = 30, midpoint_code = 513;
constexpr unsigned microvolts_per_step = 3000, tail_periods = 17, pending_periods = 39;
constexpr unsigned minimum_clock = 500000, fast_clock = 2000000, slow_clock = 1000000;
constexpr unsigned low_supply_uv = 1800000, low_supply_code = 876;

void configure(Hcs08Adc &device, const std::uint8_t control = unsigned_right) {
  device.observe_reference(reference);
  device.observe_input(0, midpoint_uv);
  device.write({.address = adc::pins, .value = 1});
  device.write({.address = adc::control, .value = control});
}

void reset_registers() {
  require(rejected([] { const Hcs08Adc absent; }).contains("explicit candidate"),
          "An unspecified chip or analog contract cannot create a successful converter");
  Hcs08Adc device(contract);
  require(device.state() == Hcs08AdcState{} && device.read(adc::control) == 0 &&
              device.read(adc::status) == 1 && device.read(adc::pins) == 0 &&
              !device.interrupt_requested(),
          "Independent ATDC/ATDSC/ATDPE reset values power down without CCF or IRQ");
  const auto initial = device.state();
  device.write({.address = adc::high, .value = byte_mask});
  device.write({.address = adc::low, .value = byte_mask});
  device.advance_bus(single_periods);
  require(device.state() == initial, "Read-only result writes and idle clocks cannot inject data");
  require(rejected([&] { static_cast<void>(device.read(adc::high)); }).contains("no qualified"),
          "Unspecified reset result data cannot become observed zero");
  device.reset();
  device.write({.address = adc::status, .value = byte_mask});
  require(device.read(adc::status) == status_mask && !device.state().active,
          "Read-only CCF stays clear and a powered-down status write cannot start conversion");
  device.reset();
  require(
      rejected([&] { device.write({.address = adc::pins + 1, .value = 0}); }).contains("reserved"),
      "The register map cannot silently extend into an unsupported peripheral");
}

void result_formats() {
  constexpr std::array<unsigned, 5> codes{0, 1, 511, 512, 1023};
  constexpr std::array<std::array<unsigned, 5>, 8> words{
      {{0x0000, 0x0040, 0x7fc0, 0x8000, 0xffc0},
       {0x8000, 0x8040, 0xffc0, 0x0000, 0x7fc0},
       {0x0000, 0x0000, 0x7f00, 0x8000, 0xff00},
       {0x8000, 0x8000, 0xff00, 0x0000, 0x7f00},
       {0x0000, 0x0001, 0x01ff, 0x0200, 0x03ff},
       {0x0000, 0x0001, 0x01ff, 0x0200, 0x03ff},
       {0x0000, 0x0000, 0x7f00, 0x8000, 0xff00},
       {0x0000, 0x0000, 0x7f00, 0x8000, 0xff00}}};
  for (unsigned mode = 0; mode < words.size(); ++mode) {
    for (unsigned n = 0; n < codes.size(); ++n) {
      Hcs08Adc device(contract);
      configure(device, static_cast<std::uint8_t>(adc::power | (mode << 4U)));
      device.observe_input(0, codes[n] * microvolts_per_step);
      device.write({.address = adc::status, .value = active_irq});
      const auto active = device.state();
      for (unsigned poll = 0; poll < 4; ++poll) {
        require(device.read(adc::status) == active_irq && device.state() == active,
                "Status polling never consumes a period or creates a sample");
      }
      device.advance_bus(single_periods - 1);
      require(!device.state().result && !device.interrupt_requested(),
              "No data or IRQ exists one BUSCLK before documented SC completion");
      device.advance_bus(1);
      require(device.read(adc::status) == ready_irq && device.interrupt_requested() &&
                  device.state().result == words[mode][n] && device.state().conversions == 1,
              "Independent literal data-format table covers justification, RES8 and ignored SGN");
      require(device.read(adc::low) == (words[mode][n] & byte_mask) &&
                  !device.interrupt_requested() && device.read(adc::high) == words[mode][n] >> 8U,
              "Either result read clears CCF/IRQ without discarding the other result byte");
    }
  }
}

void ideal_boundaries() {
  constexpr std::array<unsigned, 8> voltages{
      0, 1499, 1500, 1501, 3067499, 3067500, 3072000, 3300000};
  constexpr std::array<unsigned, 8> codes{0, 0, 1, 1, 1022, 1023, 1023, 1023};
  for (unsigned n = 0; n < voltages.size(); ++n) {
    Hcs08Adc device(contract);
    configure(device);
    device.observe_input(0, voltages[n]);
    device.write({.address = adc::status, .value = 0});
    device.advance_bus(single_periods);
    require(device.state().result == codes[n],
            "Independent half-LSB transition and full-scale saturation follow equations14-7/8");
  }
  for (unsigned code = 0; code <= adc::maximum; ++code) {
    Hcs08Adc device(contract);
    configure(device);
    device.observe_input(0, code * microvolts_per_step);
    device.write({.address = adc::status, .value = 0});
    device.advance_bus(single_periods);
    require(device.state().result == code, "All1024 own ideal input levels map independently");
  }
  for (unsigned channel = 0; channel <= adc::channel_mask; ++channel) {
    Hcs08Adc device(contract);
    configure(device);
    device.write({.address = adc::pins, .value = 0});
    device.write({.address = adc::status, .value = static_cast<std::uint8_t>(channel)});
    device.advance_bus(single_periods);
    require(device.state().result == (channel == reference_high_channel ? adc::maximum : 0),
            "Disabled pins and reserved/VREFL selections use reference low; VREFH uses full scale");
  }
}

void clocks_and_copies() {
  constexpr std::array<unsigned, 16> divisors{
      2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32};
  constexpr std::array<unsigned, 16> single{
      60, 118, 176, 234, 292, 350, 408, 466, 524, 582, 640, 698, 756, 814, 872, 930};
  for (unsigned prescaler = 0; prescaler < divisors.size(); ++prescaler) {
    Hcs08Adc device(contract);
    configure(device, static_cast<std::uint8_t>(unsigned_right | prescaler));
    auto observed = reference;
    observed.bus_hz = divisors[prescaler] * minimum_clock;
    device.observe_reference(observed);
    device.write({.address = adc::status, .value = 0});
    require(device.clock_divisor() == divisors[prescaler] &&
                device.state().remaining == single[prescaler],
            "All sixteen prescalers and first-conversion lengths match independent primary tables");
    device.advance_bus(single[prescaler]);
    require(device.state().result == midpoint_code && device.state().conversions == 1,
            "Every legal minimum-rate conversion produces the actual stable sample");
  }
  for (unsigned cut = 0; cut < single_periods; ++cut) {
    Hcs08Adc original(contract);
    configure(original);
    original.write({.address = adc::status, .value = active_irq});
    original.advance_bus(cut);
    const auto before = original.state();
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto fresh = original;
      fresh.advance_bus(single_periods - cut);
      require(fresh.state().result == midpoint_code && fresh.state().conversions == 1 &&
                  fresh.interrupt_requested() && original.state() == before,
              "Every partial-period trusted copy has independent progress, result and IRQ state");
    }
  }
}

void continuous_and_abort() {
  Hcs08Adc original(contract);
  configure(original);
  original.write({.address = adc::status, .value = active_irq | adc::continuous});
  original.advance_bus(single_periods);
  require(original.state().remaining == continuous_periods && original.state().conversions == 1,
          "Only the first continuous conversion includes documented startup overhead");
  static_cast<void>(original.read(adc::high));
  original.advance_bus(continuous_periods - 1);
  require(!original.interrupt_requested(), "A pending continuous sample cannot set CCF early");
  original.advance_bus(1);
  require(original.interrupt_requested() && original.state().conversions == 2,
          "The next real conversion reasserts CCF and its enabled IRQ");
  auto batched = original;
  constexpr unsigned count = 257;
  batched.advance_bus((continuous_periods * count) + tail_periods);
  for (unsigned n = 0; n < count; ++n) { original.advance_bus(continuous_periods); }
  original.advance_bus(tail_periods);
  require(original.state() == batched.state() && original.state().conversions == count + 2 &&
              original.state().remaining == pending_periods,
          "Batched BUSCLK uses exact subsequent-conversion lengths without poll-driven progress");
  auto stopped = original;
  stopped.write({.address = adc::pins, .value = 1});
  require(!stopped.state().active && stopped.state().complete &&
              stopped.state().result == midpoint_code,
          "ATDPE write aborts without clearing prior CCF or manufacturing another result");
  stopped.write({.address = adc::status, .value = 0});
  require(!stopped.state().complete && stopped.state().remaining == single_periods,
          "A real ATDSC write clears CCF and restarts a first conversion");
  stopped.write({.address = adc::control, .value = unsigned_right});
  require(!stopped.state().active && stopped.state().result == midpoint_code,
          "Even an identical ATDC write aborts, preserving last result data");
  const auto prior_format = stopped.state();
  require(rejected([&] {
            stopped.write({.address = adc::control, .value = adc::power});
          }).contains("reformatting") &&
              stopped.state() == prior_format,
          "An unqualified existing-result format change refuses before mutation");
  stopped.reset();
  require(
      !stopped.state().reference && !stopped.state().result &&
          stopped.state().inputs[0] == midpoint_uv && stopped.read(adc::status) == 1,
      "Reset cancels jobs/IRQ and clock qualification while retaining external pin observation");
}

void clock_boundary(const unsigned prescaler, const bool low_supply, const bool above) {
  Hcs08Adc device(contract);
  configure(device, static_cast<std::uint8_t>(unsigned_right | prescaler));
  auto observed = reference;
  const unsigned divisor = (prescaler + 1) * 2;
  const unsigned maximum = low_supply ? slow_clock : fast_clock;
  const unsigned upper = std::min(maximum_bus, divisor * maximum);
  const unsigned lower = divisor * minimum_clock;
  observed.supply_uv = low_supply ? low_supply_uv : reference.supply_uv;
  observed.high_uv = low_supply ? observed.supply_uv : reference.high_uv;
  observed.bus_hz = above ? upper : lower;
  auto valid = device;
  valid.observe_reference(observed);
  valid.write({.address = adc::status, .value = 0});
  valid.advance_bus(valid.state().remaining);
  require(valid.state().conversions == 1 &&
              valid.state().result == (low_supply ? low_supply_code : midpoint_code),
          "Both legal clock endpoints preserve the independent voltage conversion");
  observed.bus_hz = above ? upper + 1 : lower - 1;
  const auto error = rejected([&] {
    device.observe_reference(observed);
    device.write({.address = adc::status, .value = 0});
    device.advance_bus(1);
  });
  require(error.contains("outside documented") && !device.state().result,
          "Clock/supply endpoint violations cannot create valid ADC results");
}

void clock_boundaries() {
  constexpr unsigned prescalers = 16;
  for (unsigned prescaler = 0; prescaler < prescalers; ++prescaler) {
    for (const bool low_supply : {false, true}) {
      for (const bool above : {false, true}) { clock_boundary(prescaler, low_supply, above); }
    }
  }
}

void refusals() {
  for (unsigned missing = 0; missing < 3; ++missing) {
    Hcs08Adc device(contract);
    device.context({.pc = pc, .ccr = flags});
    if (missing != 0) { device.observe_reference(reference); }
    if (missing == 2) { device.observe_input(0, reference.supply_uv + 1); }
    device.write({.address = adc::pins, .value = 1});
    device.write({.address = adc::control, .value = unsigned_right});
    device.write({.address = adc::status, .value = 0});
    const auto before = device.state();
    const auto error = rejected([&] { device.advance_bus(1); });
    require(error.contains("address=0x0051 pc=0x1234 ccr=0x0068") && device.state() == before &&
                device.faulted(),
            "Unknown reference/pin and invalid voltage fail before consuming an ADC period");
    require(rejected([&] { device.advance_bus(1); }).contains("coordinated recovery"),
            "A failed analog observation cannot be silently retried");
  }
  Hcs08Adc device(contract);
  configure(device);
  device.write({.address = adc::status, .value = 0});
  const auto before = device.state();
  require(
      rejected([&] { device.observe_input(0, 0); }).contains("inactive") &&
          device.state() == before,
      "This explicit stable-window contract cannot silently accept an in-flight waveform change");
}
} // namespace

int main() {
  try {
    reset_registers();
    result_formats();
    ideal_boundaries();
    clocks_and_copies();
    continuous_and_abort();
    clock_boundaries();
    refusals();
    std::cout
        << "HCS08 ADC registers, stable analog inputs and real BUS-period conversions pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
