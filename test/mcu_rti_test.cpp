// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>

namespace {
// Independent timing reference: S12CRGUV1 table 3-2 and sections 3.1.6/4.2.
constexpr unsigned control = 0x3b;
constexpr unsigned flags = 0x37;
constexpr unsigned interrupts = 0x38;
constexpr unsigned rti_flag = 0x80;
constexpr unsigned all_flags = 0xd0;
constexpr unsigned firmware_rate = 0x40;
constexpr unsigned firmware_period = 8192;
constexpr unsigned usb_control = 0x300;
constexpr unsigned clock_select = 0x39;
constexpr unsigned reference_divider = 0x35;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void test_all_rates_and_boundaries() {
  constexpr std::array<unsigned, 8> prescaled{0, 1024, 2048, 4096, 8192, 16384, 32768, 65536};
  for (unsigned rate = 0; rate < 128; ++rate) {
    blofeld::McuClock clock;
    clock.write({.address = flags, .value = all_flags});
    clock.write({.address = control, .value = static_cast<std::uint8_t>(rate | rti_flag)});
    require(clock.read(control) == rate, "RTICTL bit 7 is reserved; other bits read back");
    const auto period = prescaled.at(rate / 16) * ((rate % 16) + 1);
    require(clock.oscillator_cycles_until_rti() == period, "Literal reference-table RTI rate");
    if (period == 0) {
      clock.advance_oscillator(std::numeric_limits<unsigned>::max());
      require(clock.read(flags) == 0 && clock.oscillator_cycles_until_rti() == 0,
              "All prescale-zero settings stop RTI without inventing timeouts");
      continue;
    }
    clock.advance_oscillator(period - 1);
    require(clock.read(flags) == 0 && clock.oscillator_cycles_until_rti() == 1,
            "No early RTI flag one oscillator cycle before each boundary");
    clock.advance_oscillator(0);
    require(clock.read(flags) == 0, "Zero clock advance cannot cause a timeout");
    clock.advance_oscillator(1);
    require(clock.read(flags) == rti_flag && clock.oscillator_cycles_until_rti() == period,
            "The exact timeout sets RTIF and immediately restarts the period");
    clock.write({.address = flags, .value = rti_flag});
    clock.advance_oscillator((period * 3) + 1);
    require(clock.read(flags) == rti_flag && clock.oscillator_cycles_until_rti() == period - 1,
            "Multiple elapsed periods retain one sticky flag and exact fractional phase");
  }
}

void test_writes_restart_and_do_not_acknowledge() {
  blofeld::McuClock clock;
  clock.write({.address = flags, .value = all_flags});
  clock.write({.address = control, .value = firmware_rate});
  clock.advance_oscillator(firmware_period - 1);
  clock.write({.address = control, .value = firmware_rate});
  require(clock.oscillator_cycles_until_rti() == firmware_period,
          "Even an unchanged RTICTL write restarts the complete timeout period");
  clock.advance_oscillator(firmware_period);
  require(!clock.realtime_interrupt_pending(), "RTIF latches while RTIE is masked");
  clock.write({.address = interrupts, .value = rti_flag});
  require(clock.realtime_interrupt_pending(), "Enabling RTIE exposes an already pending timeout");
  clock.write({.address = control, .value = 0});
  clock.advance_oscillator(std::numeric_limits<unsigned>::max());
  require(clock.realtime_interrupt_pending() && clock.oscillator_cycles_until_rti() == 0,
          "Disabling the counter does not acknowledge its pending flag");
  clock.write({.address = flags, .value = 0});
  require(clock.realtime_interrupt_pending(), "Writing zero leaves RTIF set");
  clock.write({.address = flags, .value = rti_flag});
  require(!clock.realtime_interrupt_pending() && clock.read(flags) == 0,
          "RTIF W1C acknowledges the interrupt without restarting the counter");
  constexpr unsigned changed_rate = 0x21;
  constexpr unsigned changed_period = 4096;
  clock.write({.address = control, .value = firmware_rate});
  clock.advance_oscillator(15);
  clock.write({.address = control, .value = changed_rate});
  require(clock.oscillator_cycles_until_rti() == changed_period,
          "A different modulus/prescaler starts its own full period");
  clock.advance_oscillator(changed_period - 1);
  require(!clock.realtime_interrupt_pending(),
          "An old partial period cannot shorten the new period");
  clock.advance_oscillator(1);
  require(clock.realtime_interrupt_pending(), "The new period expires at its exact boundary");
  clock.reset();
  require(
      clock.read(control) == 0 && clock.oscillator_cycles_until_rti() == 0 &&
          !clock.realtime_interrupt_pending() && clock.read(flags) == 0,
      "External reset clears RTI control, phase, enable and flag without restoring cleared PORF");
}

void test_large_advances_and_clock_domains() {
  blofeld::McuClock clock;
  constexpr unsigned rate = 0x1f;
  constexpr unsigned period = 16384;
  constexpr unsigned initial_advance = 123;
  constexpr unsigned remaining_after_large_advance = 16262;
  clock.write({.address = flags, .value = all_flags});
  clock.write({.address = control, .value = rate});
  clock.advance_oscillator(initial_advance);
  clock.advance_oscillator(std::numeric_limits<std::uint32_t>::max());
  require(clock.oscillator_cycles_until_rti() == remaining_after_large_advance,
          "UINT32_MAX clock slice preserves phase without overflow or iteration over every cycle");
  clock.write({.address = flags, .value = rti_flag});
  clock.advance_oscillator(remaining_after_large_advance - 1);
  require(clock.read(flags) == 0, "Large advances leave an exact, usable subsequent deadline");
  clock.advance_oscillator(1);
  require(clock.read(flags) == rti_flag && clock.oscillator_cycles_until_rti() == period,
          "The next exact boundary after a large advance is unchanged");
  clock.write({.address = control, .value = firmware_rate});
  constexpr unsigned release_phy = 0x40;
  clock.write({.address = usb_control, .value = release_phy});
  clock.observe_phy({.edges = 15});
  clock.write({.address = reference_divider, .value = 15});
  clock.write({.address = clock_select, .value = rti_flag});
  clock.observe_phy({.edges = 14});
  require(clock.oscillator_cycles_until_rti() == firmware_period,
          "PHY windows, PLL selection/loss and REFDV writes do not supply oscillator elapsed time");
  clock.advance_oscillator(firmware_period - 1);
  clock.write({.address = flags, .value = all_flags});
  require(clock.oscillator_cycles_until_rti() == 1,
          "Flag writes do not reset the RTI divider phase");
}

void test_bus_domain_separation_and_relocation() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus bus(flash);
  bus.write(control, firmware_rate);
  bus.advance_sci(firmware_period);
  bus.advance_timer(firmware_period);
  bus.retire_instruction();
  require(bus.oscillator_cycles_until_rti() == firmware_period,
          "SCI/timer bus cycles and instruction retirement do not fabricate oscillator cycles");
  constexpr unsigned initrg = 0x11;
  constexpr unsigned relocated = 0x1000;
  bus.write(initrg, 0x10);
  bus.retire_instruction();
  bus.advance_oscillator(firmware_period);
  require(bus.read(relocated + control) == firmware_rate &&
              (bus.read(relocated + flags) & rti_flag) != 0,
          "Relocation preserves counter state and moves RTI control/flags");
  bus.write(relocated + flags, rti_flag);
  require((bus.read(relocated + flags) & rti_flag) == 0 &&
              bus.oscillator_cycles_until_rti() == firmware_period,
          "Relocated W1C acknowledges without resetting the period");
  bus.reset();
  require(bus.read(control) == 0 && bus.oscillator_cycles_until_rti() == 0,
          "Board reset disables RTI and restores its register location");
}

void test_timeout_through_real_cpu_isr() {
  constexpr unsigned entry = 0x8000;
  constexpr unsigned handler = 0x9000;
  constexpr unsigned vector = 0xfff0;
  constexpr unsigned mailbox = 0x1200;
  std::array<std::uint8_t, 32768> flash{};
  constexpr unsigned nop = 0xa7;
  flash[0] = nop;
  flash[vector - entry] = handler >> 8U;
  // Own ISR: LDAA #$80; STAA CRGFLG; INC $1200; RTI.
  constexpr std::array<std::uint8_t, 8> code{0x86, 0x80, 0x5a, 0x37, 0x72, 0x12, 0, 0x0b};
  std::ranges::copy(code, flash.begin() + (handler - entry));
  blofeld::McuBus bus(flash);
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  bus.write(control, firmware_rate);
  bus.write(interrupts, rti_flag);
  bus.write(mailbox, 0);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  constexpr blofeld::CpuState initial{
      .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = 0x2000, .ccr = 0x40};
  cpu.set_state(initial);
  bus.advance_oscillator(firmware_period - 1);
  require(!cpu.pending_interrupt(), "No interrupt before the oscillator timeout");
  bus.advance_oscillator(1);
  const auto pending = cpu.pending_interrupt();
  require(pending && pending->vector == vector,
          "Oscillator timeout selects the literal UF32 RTI vector");
  const auto accepted = cpu.step();
  require(!accepted.instruction_executed && accepted.interrupt_vector == vector &&
              (bus.read(flags) & rti_flag) != 0,
          "Entry fetches the real vector and preserves RTIF until firmware acknowledgment");
  for (unsigned index = 0; index < 3; ++index) { cpu.step(); }
  const auto returned = cpu.step();
  require(!returned.interrupt_vector && !cpu.pending_interrupt() && bus.read(mailbox) == 1 &&
              cpu.state().pc == initial.pc && cpu.state().sp == initial.sp &&
              cpu.state().d == initial.d && cpu.state().ccr == initial.ccr,
          "Real ISR clears RTIF, increments its mailbox, and RTI restores interrupted registers");
  bus.advance_oscillator(firmware_period);
  const auto repeated = cpu.step();
  require(!repeated.instruction_executed && repeated.interrupt_vector == vector,
          "A later oscillator period generates a new interrupt after the old one was acknowledged");
}
}

int main() {
  try {
    test_all_rates_and_boundaries();
    test_writes_restart_and_do_not_acknowledge();
    test_large_advances_and_clock_domains();
    test_bus_domain_separation_and_relocation();
    test_timeout_through_real_cpu_isr();
    std::cout << "RTI divider, oscillator phases, flags and CPU ISR checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
