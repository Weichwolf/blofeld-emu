// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/clock.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using blofeld::vi::mcu::Clock;
constexpr std::uint16_t divider = 0x35, flags = 0x37, interrupts = 0x38, select = 0x39;
constexpr std::uint16_t rti_control = 0x3b, cop_control = 0x3c, test_control = 0x3e;
constexpr std::uint16_t usb_control = 0x300, rti_vector = 0xfff0, lock_vector = 0xffc6;
constexpr std::uint16_t usb_clock = 0x4000, usb_suspended = 0x4800, usb_enabled = 0xc000;
constexpr std::uint8_t power_on = 0x40, pll = 0x80, lock_change = 0x10, locked = 8;
constexpr std::uint8_t rti = 0x80, rti_fast = 0x10, all_bits = 0xff;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::uint8_t read(const Clock &clock, const std::uint16_t offset) {
  std::uint8_t value = 0;
  require(clock.read(offset, value), "Read a documented normal-mode clock register");
  return value;
}

void write(Clock &clock, const Clock::Write access) {
  require(clock.write(access), "Write a supported clock register");
}

void reset_and_qualification() {
  constexpr Clock::Cycle qualified_at = 20;
  constexpr Clock::Cycle after_pll = 40;
  constexpr Clock::Cycle after_divider = 160;
  constexpr Clock::Cycle projected_ticks = 100;
  constexpr unsigned divided_bus_period = 40;
  Clock clock;
  require(clock.cycles() == 0 && clock.at_ticks(1) == 1 && clock.at_ticks(projected_ticks) == 10,
          "Input timestamps project to the next complete oscillator bus cycle");
  require(read(clock, flags) == power_on && read(clock, usb_control) == 8 &&
              read(clock, usb_control + 1) == 0x20 && clock.bus_period() == 10 &&
              !clock.deadline() && !clock.pending(),
          "Reset selects the 12 MHz oscillator, suspended PHY and 6 MHz bus");
  require(read(clock, test_control) == pll, "Normal CTCTL reads its documented constant");
  write(clock, {.offset = usb_control, .value = usb_clock, .bytes = 2});
  require(read(clock, flags) == power_on && clock.deadline() == 2,
          "Enabling the clock does not fabricate immediate LOCK");
  require(clock.advance(1) && (read(clock, flags) & locked) == 0,
          "One bus cycle is shorter than the three-reference-period qualification");
  require(clock.advance(2) && clock.ticks() == qualified_at &&
              read(clock, flags) == (power_on | lock_change | locked) && !clock.deadline(),
          "Fifteen nominal PHY periods in three oscillator periods qualify LOCK");
  write(clock, {.offset = interrupts, .value = lock_change});
  require(clock.pending() == lock_vector, "LOCKIF and LOCKIE request the real vector");
  write(clock, {.offset = flags, .value = lock_change | power_on | locked});
  require(read(clock, flags) == locked && !clock.pending(),
          "W1C clears flags but cannot write the LOCK status");
  write(clock, {.offset = select, .value = pll});
  require(clock.bus_period() == 2 && clock.advance(12) && clock.ticks() == after_pll,
          "PLL selection changes the nominal bus to 30 MHz, independent of host time");
  require(clock.cycles() == 12 && clock.at_ticks(after_pll) == 12 &&
              clock.at_ticks(after_pll + 5) == 15,
          "Timestamp projection uses the current bus period and rounds future edges upwards");
  write(clock, {.offset = divider, .value = 15});
  require(read(clock, divider) == 0, "REFDV cannot change while PLLSEL is set");
  write(clock, {.offset = select, .value = 0});
  write(clock, {.offset = divider, .value = 3});
  require(clock.bus_period() == divided_bus_period && clock.advance(15) &&
              clock.ticks() == after_divider,
          "Reference-divider changes preserve elapsed time and affect subsequent spans");
}

void clock_switch_during_span() {
  Clock clock;
  write(clock, {.offset = usb_control, .value = usb_clock, .bytes = 2});
  write(clock, {.offset = select, .value = pll});
  require(clock.bus_period() == 10, "PLLSEL falls back to the oscillator until LOCK");
  require(clock.advance(5) && clock.bus_period() == 2 && clock.ticks() == (2 * 10) + (3 * 2),
          "A long span uses two oscillator bus periods then three qualified PHY periods");
  write(clock, {.offset = usb_control, .value = usb_suspended, .bytes = 2});
  require((read(clock, flags) & locked) != 0,
          "SPHY cannot suspend the selected PHY clock while PLLSEL is set");
  write(clock, {.offset = select, .value = 0});
  write(clock, {.offset = usb_control, .value = usb_suspended, .bytes = 2});
  require((read(clock, flags) & locked) == 0 && !clock.deadline(),
          "A legal PHY suspend removes LOCK and cancels qualification");
}

void periodic_interrupt() {
  Clock clock;
  write(clock, {.offset = rti_control, .value = rti_fast});
  write(clock, {.offset = interrupts, .value = rti});
  constexpr Clock::Cycle oscillator_bus_periods = 512;
  require(clock.deadline() == oscillator_bus_periods && clock.advance(oscillator_bus_periods - 1) &&
              !clock.pending(),
          "RTI divides 1024 oscillator periods, not CPU or host instructions");
  require(clock.advance(oscillator_bus_periods) && clock.pending() == rti_vector &&
              !clock.deadline(),
          "RTIF latches at the first real-time deadline without repeated scheduler returns");
  constexpr Clock::Cycle pending_time = 1000;
  require(clock.advance(pending_time), "Advance while the periodic flag is already pending");
  write(clock, {.offset = flags, .value = rti});
  constexpr Clock::Cycle second_deadline = 1024;
  require(clock.deadline() == second_deadline, "Clearing RTIF does not restart the divider");
  write(clock, {.offset = rti_control, .value = rti_fast});
  constexpr Clock::Cycle restarted = 1512;
  require(clock.deadline() == restarted, "Writing RTICTL restarts the divider");

  Clock pll_clock;
  write(pll_clock, {.offset = usb_control, .value = usb_clock, .bytes = 2});
  require(pll_clock.advance(2), "Complete nominal clock qualification");
  write(pll_clock, {.offset = select, .value = pll});
  write(pll_clock, {.offset = rti_control, .value = rti_fast});
  write(pll_clock, {.offset = interrupts, .value = rti | lock_change});
  constexpr Clock::Cycle pll_deadline = 2562;
  require(pll_clock.deadline() == pll_deadline && pll_clock.advance(pll_deadline - 1) &&
              pll_clock.pending() == lock_vector,
          "The faster CPU bus does not speed up oscillator-clocked RTI");
  require(pll_clock.advance(pll_deadline) && pll_clock.pending() == rti_vector,
          "RTI has priority over the simultaneous PLL-lock interrupt");
}

void wait_and_stop() {
  Clock clock;
  write(clock, {.offset = rti_control, .value = rti_fast});
  write(clock, {.offset = interrupts, .value = rti});
  constexpr Clock::Cycle initial = 100;
  constexpr Clock::Cycle after_wait = 1100;
  constexpr Clock::Cycle after_stop = 1200;
  require(clock.advance(initial), "Accumulate a partial RTI period");
  write(clock, {.offset = select, .value = 2});
  clock.activity(Clock::Activity::waiting);
  require(clock.bus_running() && !clock.deadline() && clock.advance(after_wait) && !clock.pending(),
          "RTIWAI initializes and freezes RTI without stopping the IP bus");
  clock.activity(Clock::Activity::running);
  constexpr Clock::Cycle resumed = 1612;
  require(clock.deadline() == resumed, "RTIWAI resumes with a complete new period");
  write(clock, {.offset = select, .value = 0x20});
  clock.activity(Clock::Activity::waiting);
  require(!clock.bus_running() && clock.deadline() == resumed,
          "SYSWAI gates the bus but preserves the independent RTI wake source");
  clock.activity(Clock::Activity::stopped);
  require(!clock.deadline() && clock.advance(after_stop) && !clock.pending(),
          "STOP freezes RTI even when its interrupt is enabled");
}

void controls_and_limits() {
  Clock clock;
  constexpr std::uint8_t disabled_cop = 0xc0;
  write(clock, {.offset = cop_control, .value = disabled_cop});
  write(clock, {.offset = cop_control, .value = all_bits});
  require(read(clock, cop_control) == disabled_cop, "Normal-mode COPCTL is write-once");
  Clock unsupported;
  require(!unsupported.write({.offset = cop_control, .value = 1}) &&
              read(unsupported, cop_control) == 0,
          "An unbound watchdog reset path cannot be silently enabled");
  write(clock, {.offset = usb_control, .value = usb_clock, .bytes = 2});
  constexpr std::uint16_t endpoint_reset = 0x4200;
  require(!clock.write({.offset = usb_control, .value = endpoint_reset, .bytes = 2}) &&
              read(clock, usb_control) == power_on,
          "Endpoint reset is refused until its USB owner is connected");
  write(clock, {.offset = usb_control, .value = usb_enabled, .bytes = 2});
  write(clock, {.offset = usb_control, .value = 0x80});
  require(read(clock, usb_control) == (usb_enabled >> 8U), "MCE cannot change while MEN is set");
  write(clock, {.offset = select, .value = 1});
  write(clock, {.offset = select, .value = 0});
  require(read(clock, select) == 1, "Normal-mode COPWAI is write-once independently of PLLSEL");

  Clock large;
  write(large, {.offset = rti_control, .value = rti_fast});
  constexpr Clock::Cycle many = 1'000'000'000'000;
  require(large.advance(many) && large.ticks() == many * 10 && (read(large, flags) & rti) != 0,
          "Large spans latch periodic flags in constant time");
  require(!large.advance(many - 1) && !large.advance(std::numeric_limits<Clock::Cycle>::max()),
          "Backward time and unrepresentable nominal timestamps fail explicitly");
  std::uint8_t value = 0;
  require(!large.read(0x40, value) && !large.write({.offset = 0x40}) &&
              !large.write({.offset = select, .bytes = 3}),
          "Clock ownership does not swallow timer registers or invalid widths");
}
}

int main() try {
  reset_and_qualification();
  clock_switch_during_span();
  periodic_interrupt();
  wait_and_stop();
  controls_and_limits();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
