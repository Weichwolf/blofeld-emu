// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_clock.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned timebase_hz = 60000000;
constexpr unsigned reference_divider = 0x35;
constexpr unsigned clock_select = 0x39;
constexpr unsigned usb_high = 0x300;
constexpr unsigned phy_start = 0x40;
constexpr unsigned pll_select = 0x80;
constexpr unsigned flags = 0xc0;
constexpr unsigned pc = 0x8123;
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned realtime_vector = 0xfff0;
constexpr unsigned mailbox = 0x1200;
constexpr unsigned return_pc = 0x800c;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected strict run-clock rejection");
}

std::array<std::uint8_t, 32768> blank_flash() {
  return {};
}

blofeld::McuRunClock reference_clock(const std::uint32_t rate) {
  return blofeld::McuRunClock(
      {.oscillator_hz = rate,
       .initial_oscillator_phase_ticks = 0,
       .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary});
}

void select_phy(blofeld::McuBus &bus) {
  bus.write(clock_select, 0);
  bus.write(usb_high, phy_start);
  bus.observe_phy({.edges = 15});
  bus.write(clock_select, pll_select);
}

void tick(blofeld::McuRunClock &clock, blofeld::McuBus &bus) {
  clock.advance_run_cycle(bus, {.pc = pc, .ccr = flags});
}

void test_all_reference_dividers() {
  // Literal S12CRGUV1 divider values with the device's two-phase CPU bus:
  // OSCCLK=12 MHz => 10..160 ticks at 60 MHz; PHY=60 MHz => 2..32 ticks.
  constexpr std::array<unsigned, 16> divisors{
      2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32};
  constexpr std::array<unsigned, 16> oscillator_ticks{
      10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160};
  for (unsigned index = 0; index < divisors.size(); ++index) {
    blofeld::McuBus bus(blank_flash());
    auto clock = reference_clock(oscillator_hz);
    bus.write(reference_divider, static_cast<std::uint8_t>(index));
    bus.write(clock_select, pll_select); // Known unlocked PHY falls back to OSC.
    const auto fallback = bus.run_clock_route();
    require(fallback.source == blofeld::McuClockSource::oscillator &&
                fallback.bus_divider == divisors.at(index),
            "LOCK=0 selects the literal oscillator bus divider even with PLLSEL=1");
    require(clock.next_cycle_ticks(bus) == oscillator_ticks.at(index) && clock.bus_cycles() == 0,
            "The board duration offer preserves the literal source/divider without advancing time");
    tick(clock, bus);
    require(clock.timebase_hz() == timebase_hz && clock.bus_cycles() == 1 &&
                clock.elapsed_ticks() == oscillator_ticks.at(index) &&
                clock.oscillator_cycles() == divisors.at(index),
            "Each oscillator-selected bus cycle contains the independently specified edges");
    select_phy(bus);
    const auto selected = bus.run_clock_route();
    require(selected.source == blofeld::McuClockSource::usb_phy &&
                selected.bus_divider == divisors.at(index),
            "Qualified PHY selects the same software divider and fixed 60 MHz source");
    require(clock.next_cycle_ticks(bus) == divisors.at(index) && clock.bus_cycles() == 1,
            "A changed native source affects the pure next-period offer without consuming it");
    for (unsigned count = 0; count < 5; ++count) { tick(clock, bus); }
    require(clock.bus_cycles() == 6 &&
                clock.elapsed_ticks() == oscillator_ticks.at(index) + (5 * divisors.at(index)) &&
                clock.oscillator_cycles() == 2 * static_cast<std::uint64_t>(divisors.at(index)),
            "Five PHY bus cycles preserve the exact 12/60 oscillator ratio for every divider");
  }
}

void test_phase_across_switch_and_copy() {
  blofeld::McuBus bus(blank_flash());
  auto clock = reference_clock(oscillator_hz);
  select_phy(bus);
  tick(clock, bus); // 2/5 of an oscillator cycle.
  require(clock.elapsed_ticks() == 2 && clock.oscillator_cycles() == 0,
          "The first PHY cycle retains a fractional oscillator phase");
  const auto frozen = clock;
  auto clone = bus.clone();
  bus.write(clock_select, 0);
  tick(clock, bus); // Add two whole oscillator periods, retaining the 2/5 fraction.
  bus.write(clock_select, pll_select);
  tick(clock, bus);
  tick(clock, bus);
  require(clock.elapsed_ticks() == 16 && clock.oscillator_cycles() == 3 && clock.bus_cycles() == 4,
          "Source changes preserve phase instead of resetting or rounding oscillator time");
  auto replay = frozen;
  clone->write(clock_select, 0);
  tick(replay, *clone);
  clone->write(clock_select, pll_select);
  tick(replay, *clone);
  tick(replay, *clone);
  require(replay.elapsed_ticks() == 16 && replay.oscillator_cycles() == 3 &&
              replay.bus_cycles() == 4,
          "A trusted clock/bus copy retains the unfinished fraction across subsequent switches");
  bus.observe_phy({.edges = 14});
  tick(clock, bus);
  constexpr unsigned fallback_ticks = 26;
  require(clock.elapsed_ticks() == fallback_ticks && clock.oscillator_cycles() == 5 &&
              bus.run_clock_route().source == blofeld::McuClockSource::oscillator,
          "Observed lock loss falls back to OSC while retaining the existing fraction");
}

void test_large_exact_timebase() {
  constexpr unsigned prime_oscillator = 39999989;
  constexpr std::uint64_t common_rate = 2399999340000000;
  constexpr std::uint64_t first_ticks = 1920000000;
  constexpr std::uint64_t final_ticks = 7680000000;
  constexpr unsigned divider_value = 15;
  constexpr unsigned expected_edges = 128;
  blofeld::McuBus bus(blank_flash());
  auto clock = reference_clock(prime_oscillator);
  bus.write(reference_divider, divider_value);
  tick(clock, bus);
  require(clock.timebase_hz() == common_rate && clock.elapsed_ticks() == first_ticks &&
              clock.oscillator_cycles() == 32,
          "A coprime oscillator retains an exact 64-bit timebase without floating point");
  for (unsigned count = 0; count < 3; ++count) { tick(clock, bus); }
  require(clock.elapsed_ticks() == final_ticks && clock.oscillator_cycles() == expected_edges &&
              clock.bus_cycles() == 4,
          "The exact elapsed time exceeds 32 bits without losing oscillator edges");
  select_phy(bus);
  require(rejects([&clock, &bus] { tick(clock, bus); }).contains("non-12 MHz reference") &&
              clock.faulted() && clock.elapsed_ticks() == final_ticks,
          "A nominal 60 MHz PHY and its 15/3 detector window require the nominal 12 MHz source");
}

void test_clock_failures_are_contextual_and_latched() {
  constexpr std::array<unsigned, 3> invalid_rates{0, 499999, 40000001};
  for (const auto rate : invalid_rates) {
    static_cast<void>(rejects([rate] { static_cast<void>(reference_clock(rate)); }));
  }
  blofeld::McuBus bus(blank_flash());
  auto clock = reference_clock(oscillator_hz);
  bus.write(usb_high, phy_start);
  bus.write(clock_select, pll_select);
  const auto error = rejects([&clock, &bus] { tick(clock, bus); });
  require(error.contains("MCU run clock source at 0x0039") && error.contains("pc=0x8123") &&
              error.contains("ccr=0xc0") && clock.faulted() && clock.bus_cycles() == 0 &&
              clock.elapsed_ticks() == 0,
          "Unknown PHY qualification cannot be replaced by a guessed clock source");
  bus.observe_phy({.edges = 15});
  require(rejects([&clock, &bus] { tick(clock, bus); }).contains("coordinated recovery") &&
              clock.elapsed_ticks() == 0,
          "A failed clock cannot resume by supplying only a later PHY observation");
  constexpr unsigned timer_control = 0x46;
  constexpr unsigned capture_control = 0x4b;
  blofeld::McuBus timer_bus(blank_flash());
  auto timer_clock = reference_clock(oscillator_hz);
  timer_bus.write(timer_control, pll_select);
  timer_bus.write(capture_control, 1);
  const auto timer_error = rejects([&timer_clock, &timer_bus] { tick(timer_clock, timer_bus); });
  require(timer_error.contains("MCU timer advance at 0x0040") &&
              timer_error.contains("mask=0x01") && timer_error.contains("pc=0x8123") &&
              timer_clock.faulted() && timer_clock.bus_cycles() == 0,
          "A partially advanced peripheral failure exposes its operation and latches recovery");
}

void test_explicit_clock_contract_and_initial_phase() {
  require(rejects([] {
            const blofeld::McuRunClock clock({.oscillator_hz = oscillator_hz});
          }).contains("Explicit reference-cycle clock switching contract"),
          "Unqualified source-switch timing is never silently accepted");
  require(rejects([] {
            const blofeld::McuRunClock clock(
                {.oscillator_hz = oscillator_hz,
                 .initial_oscillator_phase_ticks = 5,
                 .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary});
          }).contains("Initial oscillator phase"),
          "A phase at or beyond the next oscillator edge must be rejected");
  blofeld::McuBus bus(blank_flash());
  select_phy(bus);
  blofeld::McuRunClock clock(
      {.oscillator_hz = oscillator_hz,
       .initial_oscillator_phase_ticks = 2,
       .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary});
  tick(clock, bus);
  require(clock.elapsed_ticks() == 2 && clock.oscillator_cycles() == 0,
          "An explicit two-tick initial phase remains below the next oscillator edge");
  tick(clock, bus);
  require(clock.elapsed_ticks() == 4 && clock.oscillator_cycles() == 1,
          "The specified initial phase moves the first oscillator edge into the second cycle");
}

void test_relocated_clock_error() {
  blofeld::McuBus bus(blank_flash());
  auto clock = reference_clock(oscillator_hz);
  constexpr unsigned map_register = 0x11;
  constexpr unsigned relocated_usb = 0x0b00;
  constexpr unsigned relocated_select = 0x0839;
  bus.write(map_register, 8);
  bus.retire_instruction();
  bus.write(relocated_usb, phy_start);
  bus.write(relocated_select, pll_select);
  const auto error = rejects([&clock, &bus] { tick(clock, bus); });
  require(error.contains("MCU run clock source at 0x0839") && error.contains("pc=0x8123") &&
              error.contains("ccr=0xc0") && clock.faulted() && clock.bus_cycles() == 0,
          "Unknown source diagnostics follow the native relocated register map");
}

std::array<std::uint8_t, 32768> interrupt_flash() {
  auto flash = blank_flash();
  // LDS #2000; LDX #0400; DBNE X,self; CLI; NOP; BRA self.
  constexpr std::array<std::uint8_t, 14> main{
      0xcf, 0x20, 0, 0xce, 0x04, 0, 0x04, 0x35, 0xfd, 0x10, 0xef, 0xa7, 0x20, 0xfe};
  // LDAA #80; STAA CRGFLG; INC mailbox; RTI. Real W1C and native return.
  constexpr std::array<std::uint8_t, 9> isr{0x86, 0x80, 0x7a, 0, 0x37, 0x72, 0x12, 0, 0x0b};
  std::ranges::copy(main, flash.begin());
  std::ranges::copy(isr, flash.begin() + handler - entry);
  flash[realtime_vector - entry] = handler >> 8U;
  constexpr unsigned reset_vector = 0xfffe;
  flash[reset_vector - entry] = entry >> 8U;
  return flash;
}

void run_until_return(blofeld::Hc12 &cpu, blofeld::McuBus &bus, const unsigned value) {
  constexpr unsigned maximum_instructions = 3000;
  for (unsigned count = 0; count < maximum_instructions; ++count) {
    const auto result = cpu.step();
    require(!result.interrupt_vector || result.interrupt_vector == realtime_vector,
            "Only the independently clocked oscillator RTI may interrupt the own program");
    if (bus.read(mailbox) == value && cpu.state().pc == return_pc) { return; }
  }
  throw blofeld::Fault("Native foreground and RTI must reach a bounded real return");
}

void test_cpu_clocked_realtime_interrupts() {
  blofeld::McuBus bus(interrupt_flash());
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  auto clock = reference_clock(oscillator_hz);
  select_phy(bus);
  constexpr unsigned rti_control = 0x3b;
  constexpr unsigned rti_enable = 0x38;
  constexpr unsigned minimum_period = 0x10;
  bus.write(rti_control, minimum_period);
  bus.write(rti_enable, pll_select);
  bus.write(mailbox, 0);
  blofeld::Hc12CycleBus timed(
      bus, [&bus, &clock](const auto cycle) { clock.advance_run_cycle(bus, cycle.context); });
  blofeld::Hc12 cpu(timed, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  run_until_return(cpu, bus, 1);
  // 5 reset + LDS/LDX4 + 1024*DBNE3 + CLI/NOP2 + IRQ9 + ISR(1+3+4+8).
  constexpr unsigned first_cycles = 3108;
  constexpr unsigned first_edges = 1243;
  constexpr unsigned until_second_period = 805;
  constexpr unsigned flags_register = 0x37;
  require(timed.cycles() == first_cycles && clock.bus_cycles() == first_cycles &&
              clock.elapsed_ticks() == static_cast<std::uint64_t>(first_cycles) * 2 &&
              clock.oscillator_cycles() == first_edges &&
              bus.oscillator_cycles_until_rti() == until_second_period &&
              (bus.read(flags_register) & pll_select) == 0 && cpu.state().sp == 0x2000 &&
              cpu.state().d == 0 && cpu.state().x == 0 && cpu.state().ccr == flags,
          "CPU cycles must drive oscillator timeout, real vector/ISR/W1C and exact RTI return");
  run_until_return(cpu, bus, 2);
  // Next timeout at5120 falls within a three-cycle BRA ending5121; IRQ9+ISR16.
  constexpr unsigned second_cycles = 5146;
  constexpr unsigned second_edges = 2058;
  require(timed.cycles() == second_cycles && clock.bus_cycles() == second_cycles &&
              clock.oscillator_cycles() == second_edges && !cpu.pending_interrupt() &&
              cpu.state().sp == 0x2000 && cpu.state().ccr == flags,
          "The next free-running oscillator period enters after the actual branch boundary");
}
}

int main() {
  try {
    test_all_reference_dividers();
    test_phase_across_switch_and_copy();
    test_large_exact_timebase();
    test_clock_failures_are_contextual_and_latched();
    test_explicit_clock_contract_and_initial_phase();
    test_relocated_clock_error();
    test_cpu_clocked_realtime_interrupts();
    std::cout
        << "Exact normal-run source/divider/oscillator phase and native CPU RTI checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
