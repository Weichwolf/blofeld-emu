// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_timer.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
// Exact UF32 TIM16B8CV1 sections 1.3.2.15..17. PAEN stays off here.
// This scope does not qualify active counting, flag clearing or word bus timing.
constexpr std::uint16_t counter_high = 0x44;
constexpr std::uint16_t counter_low = 0x45;
constexpr std::uint16_t timer_control = 0x46;
constexpr std::uint16_t channel_selection = 0x40;
constexpr std::uint16_t channel_flags = 0x4e;
constexpr std::uint16_t overflow_flags = 0x4f;
constexpr std::uint16_t channel0_high = 0x50;
constexpr std::uint16_t channel0_low = 0x51;
constexpr std::uint16_t pulse_control = 0x60;
constexpr std::uint16_t pulse_flags = 0x61;
constexpr std::uint16_t pulse_high = 0x62;
constexpr std::uint16_t pulse_low = 0x63;
constexpr std::uint8_t timer_enable = 0x80;
constexpr std::uint8_t dormant_control = 0xbf;
constexpr std::uint8_t dormant_readback = 0x3f;
constexpr std::uint8_t replacement_high = 0x5c;
constexpr std::uint8_t replacement_low = 0xc3;
constexpr unsigned high_shift = 8;
constexpr unsigned low_mask = 0xff;
constexpr unsigned ticks = 7;
constexpr unsigned counter_wrap = 65536;
constexpr std::uint8_t channel0 = 1;
constexpr std::uint8_t timer_fast_clear = 0x90;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::uint16_t count(blofeld::McuTimer &timer) {
  return static_cast<std::uint16_t>((static_cast<unsigned>(timer.read(pulse_high)) << high_shift) |
                                    static_cast<unsigned>(timer.read(pulse_low)));
}

struct Configuration {
  std::uint8_t control;
  std::uint16_t value;
};

void storage(const Configuration input) {
  blofeld::McuTimer timer;
  require(count(timer) == 0 && timer.read(pulse_control) == 0 && timer.read(pulse_flags) == 0,
          "PACNT, PACTL and PAFLG reset to zero");
  timer.write({.address = timer_control, .value = input.control});
  timer.write({.address = pulse_control, .value = dormant_control});
  require(timer.read(pulse_control) == dormant_readback,
          "Reserved PACTL bit7 reads zero; dormant mode/clock/IRQ fields remain readable");
  const auto high = static_cast<std::uint8_t>(input.value >> high_shift);
  const auto low = static_cast<std::uint8_t>(input.value & low_mask);
  timer.write({.address = pulse_high, .value = high});
  require(count(timer) == (static_cast<unsigned>(high) << high_shift),
          "High-byte PACNT write leaves the independently reset low byte unchanged");
  timer.write({.address = pulse_low, .value = low});
  require(count(timer) == input.value, "PACNT retains the independent full16bit literal");
  timer.write({.address = pulse_high, .value = replacement_high});
  require(count(timer) == ((static_cast<unsigned>(replacement_high) << high_shift) | low),
          "High-byte replacement preserves the previously stored low byte");
  timer.write({.address = pulse_high, .value = high});
  timer.write({.address = pulse_low, .value = replacement_low});
  require(count(timer) == ((static_cast<unsigned>(high) << high_shift) | replacement_low),
          "Low-byte replacement preserves the previously stored high byte");
  timer.write({.address = pulse_low, .value = low});
  timer.advance(ticks);
  const auto main_counter = (static_cast<unsigned>(timer.read(counter_high)) << high_shift) |
                            static_cast<unsigned>(timer.read(counter_low));
  require(count(timer) == input.value,
          "Disabled PAEN cannot count bus clocks or change the software-written PACNT value");
  require(main_counter == ((input.control & timer_enable) != 0 ? ticks : 0),
          "Dormant PACTL CLK fields cannot replace the normal main-timer prescaler clock");
  timer.reset();
  require(count(timer) == 0 && timer.read(pulse_control) == 0 && timer.read(pulse_flags) == 0,
          "Actual timer reset clears the written PACNT and all dormant PA configuration");
}

void preserves_main_timer_flags() {
  blofeld::McuTimer timer;
  timer.write({.address = channel_selection, .value = channel0});
  timer.write({.address = channel0_high, .value = 0});
  timer.write({.address = channel0_low, .value = 1});
  timer.write({.address = timer_control, .value = timer_enable});
  timer.advance(counter_wrap);
  require(timer.read(channel_flags) == channel0 && timer.read(overflow_flags) == timer_enable,
          "Real channel0 compare and complete counter wrap establish both main-timer flags");
  timer.write({.address = timer_control, .value = timer_fast_clear});
  timer.write({.address = pulse_high, .value = replacement_high});
  timer.write({.address = pulse_low, .value = replacement_low});
  require(count(timer) ==
              ((static_cast<unsigned>(replacement_high) << high_shift) | replacement_low),
          "Software PACNT accesses preserve their written value while the main timer is active");
  require(timer.read(channel_flags) == channel0 && timer.read(overflow_flags) == timer_enable,
          "PACNT fast-clear cannot acknowledge unrelated channel or main-counter overflow flags");
}
}

int main() {
  try {
    constexpr std::array<std::uint8_t, 4> controls{0, 0x10, 0x80, 0x90};
    constexpr std::array<std::uint16_t, 8> values{
        0, 0xffff, 0xa55a, 0x0f00, 0x007f, 0x8001, 0x8234, 0xf0a5};
    for (const auto control : controls) {
      for (const auto value : values) { storage({.control = control, .value = value}); }
    }
    preserves_main_timer_flags();
    std::cout << "32 register cases with PAEN disabled and main-timer flag independence passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
