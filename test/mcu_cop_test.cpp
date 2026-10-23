// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_cop.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned control_address = 0x3c, service_address = 0x3f;
constexpr unsigned reserved_control = 0x38, masked_ccr = 0xd0;
constexpr unsigned control_mask = 0xc7, window = 0x80, arm = 0x55, finish = 0xaa;
constexpr unsigned invalid_service = 0xf0, maximum_byte = 255;
constexpr unsigned first_period = 16384, cutoff = 12288;
constexpr unsigned partial_phase = 100, relocated_phase = 500, wake_phase = 26, returned_phase = 42;
constexpr unsigned diagnostic_pc = 0x8123, rti_control_value = 0x12;
constexpr unsigned oscillator_hz = 12000000, irq_unmasked = 0x80;
constexpr unsigned held_wait = 20000, before_expiry_idle = 8184, expired_bus_cycles = 8196;
constexpr std::array<unsigned, 8> periods{
    0, 16384, 65536, 262144, 1048576, 4194304, 8388608, 16777216};

std::string error(const auto &operation) {
  try {
    operation();
  } catch (const Fault &fault) { return fault.what(); }
  throw Fault("Expected strict COP rejection");
}

void test_register_domain_and_lock() {
  for (unsigned value = 0; value <= maximum_byte; ++value) {
    McuCop cop;
    if ((value & reserved_control) != 0) {
      require(error([&] {
                cop.write_control(static_cast<std::uint8_t>(value));
              }).contains("reserved bits") &&
                  cop.state() == McuCopState{},
              "Unqualified reserved-bit writes cannot create invented hardware behavior");
      continue;
    }
    cop.write_control(static_cast<std::uint8_t>(value));
    require(cop.state().control == (value & control_mask) && cop.state().control_written &&
                cop.period() == periods.at(value & 7U),
            "Every control byte has literal normal-mode mask/rate semantics");
    cop.advance(3);
    const auto frozen = cop.state();
    cop.write_control(static_cast<std::uint8_t>(value ^ maximum_byte));
    require(cop.state() == frozen,
            "Normal-mode control is entirely write-once, including disabled zero");
    cop.reset();
    require(cop.state() == McuCopState{} && cop.remaining() == 0,
            "Reset clears counter, partial service and write-once history");
    cop.write_control(1);
    require(cop.period() == first_period,
            "External reset rearms the complete normal control latch");
  }
  const McuClock clock;
  require(clock.read(control_address) == 0 && clock.read(service_address) == 0,
          "Both physical COP register offsets read reset-zero; ARMCOP is always read-zero");
}

void test_rate_deadlines() {
  for (unsigned rate = 1; rate < periods.size(); ++rate) {
    for (const unsigned flags : {0U, 0x40U, window, window | 0x40U}) {
      McuCop cop;
      cop.write_control(static_cast<std::uint8_t>(flags | rate));
      require(cop.advance(periods.at(rate) - 1) == periods.at(rate) - 1 && cop.remaining() == 1 &&
                  cop.state().reset == McuCopReset::none,
              "All seven NXP rates expire at their exact final OSCCLK edge");
      require(cop.advance(1) == 1 && cop.state().phase == periods.at(rate) &&
                  cop.remaining() == 0 && cop.state().reset == McuCopReset::timeout,
              "Timeout latches rather than wrapping or silently restarting");
      const auto frozen = cop.state();
      require(cop.advance(std::numeric_limits<unsigned>::max()) == 0 && cop.state() == frozen,
              "Pending reset retains its first event and admits no post-reset counter state");
      require(error([&] { cop.service(arm); }).contains("requires system reset"),
              "A later service cannot undo a pending system reset");
    }
  }
}

void test_service_sequences() {
  McuCop cop;
  for (unsigned value = 0; value <= maximum_byte; ++value) {
    cop.service(static_cast<std::uint8_t>(value));
  }
  require(cop.state() == McuCopState{} && cop.advance(std::numeric_limits<unsigned>::max()) ==
                                              std::numeric_limits<unsigned>::max(),
          "Disabled service writes have the documented absence of effect");
  cop.write_control(1);
  cop.advance(partial_phase);
  cop.service(finish);
  require(cop.state().phase == partial_phase && !cop.state().armed,
          "AA without55 neither services nor resets the watchdog");
  cop.service(arm);
  cop.service(arm);
  require(cop.state().phase == partial_phase && cop.state().armed,
          "Repeated55 is allowed without restarting the counter");
  const auto half = cop;
  cop.service(finish);
  require(cop.state().phase == 0 && !cop.state().armed,
          "Only a complete55/AA sequence starts the next period");
  cop.advance(10);
  cop.service(finish);
  require(cop.state().phase == 10 && !cop.state().armed,
          "RepeatedAA does not extend the newly started period");
  auto replay = half;
  replay.advance(7);
  replay.service(finish);
  require(replay.state().phase == 0 && half.state().phase == partial_phase && half.state().armed,
          "Copies preserve partial service and remain independently mutable");
  cop.reset();
  cop.write_control(1);
  cop.service(arm);
  cop.advance(first_period);
  require(cop.state().reset == McuCopReset::timeout && cop.state().armed,
          "An incomplete55-only sequence still times out at the original deadline");
}

void test_window_edges_and_invalid_bytes() {
  for (unsigned rate = 1; rate < periods.size(); ++rate) {
    const auto start = periods.at(rate) - (periods.at(rate) / 4);
    McuCop early;
    early.write_control(static_cast<std::uint8_t>(window | rate));
    early.advance(start - 1);
    early.service(arm);
    require(early.state().reset == McuCopReset::early_service,
            "The last edge before the75-percent window is still illegal");
    McuCop exact;
    exact.write_control(static_cast<std::uint8_t>(window | rate));
    exact.advance(start);
    exact.service(finish);
    require(exact.state().phase == start && exact.state().reset == McuCopReset::none,
            "The first window edge permitsAA but does not invent a preceding55");
    exact.service(arm);
    exact.service(arm);
    exact.service(finish);
    require(exact.state().phase == 0 && exact.state().reset == McuCopReset::none,
            "A complete sequence exactly at the window edge restarts normally");
    exact.service(finish);
    require(exact.state().reset == McuCopReset::early_service,
            "Even repeatedAA must satisfy the new period's window");
    McuCop last;
    last.write_control(static_cast<std::uint8_t>(window | rate));
    last.advance(periods.at(rate) - 1);
    last.service(arm);
    last.service(finish);
    require(last.state().phase == 0 && last.state().reset == McuCopReset::none,
            "The last pre-timeout edge still permits a complete windowed service");
  }
  for (const unsigned flags : {0U, window}) {
    for (unsigned value = 0; value <= maximum_byte; ++value) {
      if (value == arm || value == finish) { continue; }
      McuCop cop;
      cop.write_control(static_cast<std::uint8_t>(flags | 1U));
      cop.advance(cutoff);
      cop.service(static_cast<std::uint8_t>(value));
      require(cop.state().reset == McuCopReset::invalid_service && cop.state().phase == cutoff,
              "All254 other service bytes request reset immediately, without advancing time");
    }
  }
}

void test_clock_deadline_and_relocation() {
  McuClock clock;
  clock.write({.address = control_address, .value = 1});
  clock.write(
      {.address = hc12::rti_control, .value = rti_control_value}); // Independent period3072.
  clock.advance_oscillator(std::numeric_limits<unsigned>::max());
  require(clock.cop_state().phase == first_period &&
              clock.cop_state().reset == McuCopReset::timeout &&
              clock.oscillator_cycles_until_rti() == 2048 &&
              (clock.read(hc12::crg_flags) & window) != 0,
          "A huge supplied slice stops every oscillator consumer at the first COP deadline");
  McuBus bus(blofeld::test::checkpoint_image());
  bus.begin_instruction({.pc = diagnostic_pc, .ccr = masked_ccr});
  bus.write(hc12::initrg, 0x10);
  bus.retire_instruction();
  constexpr unsigned relocated_control = 0x103c;
  constexpr unsigned relocated_service = 0x103f;
  bus.write(relocated_control, 1);
  bus.write(relocated_service, arm);
  bus.advance_oscillator(relocated_phase);
  auto copy = bus.clone();
  copy->write(relocated_service, finish);
  require(copy->cop_state().phase == 0 && !copy->cop_state().armed &&
              bus.cop_state().phase == relocated_phase && bus.cop_state().armed,
          "Relocated registers and native bus cloning retain the partial COP state");
  const auto fault = error([&] { copy->write(relocated_service, invalid_service); });
  require(fault.contains("MCU write at 0x103f") && fault.contains("invalid ARMCOP service") &&
              fault.contains("pc=0x8123") &&
              copy->cop_state().reset == McuCopReset::invalid_service,
          "A real reset request remains visible with relocated operation/address/CPU context");
  auto pending = copy->clone();
  require(pending->cop_state() == copy->cop_state() && pending->read(relocated_service) == 0,
          "Cloning a pending event keeps its cause rather than clearing or applying it");
  require(error([&] {
            pending->check_execute({.address = blofeld::test::entry, .length = 1});
          }).contains("COP system reset"),
          "Debug register replacement cannot execute through an unapplied system reset");
  pending->reset();
  require(pending->cop_state() == McuCopState{},
          "An explicit external reset clears the old pending cause");
}

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::internal_flash_reads};
}

void test_fractional_phy_edges() {
  McuBus bus(blofeld::test::checkpoint_image());
  McuRunClock clock(contract().clock);
  bus.write(hc12::usb_control, 0x40);
  bus.observe_phy({.edges = 15});
  bus.write(hc12::clock_select, window);
  bus.write(control_address, 1);
  for (unsigned n = 0; n < 5; ++n) { clock.advance_run_cycle(bus, {}); }
  require(clock.elapsed_ticks() == 10 && clock.oscillator_cycles() == 2 &&
              bus.cop_state().phase == 2,
          "Five60MHz-source IPBus periods contain exactly two12MHz OSCCLK edges for COP");
  auto copied_bus = bus.clone();
  auto copied_clock = clock;
  copied_bus->write(service_address, arm);
  for (unsigned n = 0; n < 3; ++n) { copied_clock.advance_run_cycle(*copied_bus, {}); }
  require(
      copied_bus->cop_state().phase == 3 && copied_bus->cop_state().armed &&
          bus.cop_state().phase == 2,
      "Bus/clock copies retain the unfinished source-clock fraction and independent service state");
}

void test_pure_deadline_offers() {
  constexpr std::array<unsigned, 5> oscillator_ticks{5, 4, 3, 2, 1};
  constexpr std::array<unsigned, 5> phy_ticks{2, 2, 2, 2, 1};
  for (unsigned phase = 0; phase < oscillator_ticks.size(); ++phase) {
    auto inputs = contract().clock;
    inputs.initial_oscillator_phase_ticks = phase;
    const McuRunClock clock(inputs);
    McuBus bus(blofeld::test::checkpoint_image());
    bus.write(control_address, 1);
    bus.advance_oscillator(first_period - 1);
    const auto before = bus.cop_state();
    const auto offer = clock.next_period(bus);
    require(offer.duration_ticks == oscillator_ticks.at(phase) && offer.cop_deadline &&
                clock.next_cycle_ticks(bus) == 10 && bus.cop_state() == before &&
                clock.elapsed_ticks() == 0 && clock.bus_cycles() == 0,
            "A pure COP offer preserves state and caps the native period at its literal OSC edge");
    bus.write(hc12::usb_control, 0x40);
    bus.observe_phy({.edges = 15});
    auto held_bus = bus.clone(); // COPWAI is write-once: qualify its first write independently.
    bus.write(hc12::clock_select, window);
    const auto phy = clock.next_period(bus);
    require(phy.duration_ticks == phy_ticks.at(phase) && phy.cop_deadline == (phase >= 3),
            "A fractional PHY offer distinguishes no deadline, exact-end timeout and partial slot");
    held_bus->write(hc12::clock_select, window | 1U);
    const auto held = clock.next_period(*held_bus, true);
    require(held.duration_ticks == 2 && !held.cop_deadline && bus.cop_state() == before,
            "COPWAI gates only native WAI offers, without consuming or clearing the counter");
  }
  const McuRunClock clock(contract().clock);
  McuBus bus(blofeld::test::checkpoint_image());
  bus.write(control_address, 1);
  bus.advance_oscillator(first_period - 2);
  require(clock.next_period(bus).duration_ticks == 10 && clock.next_period(bus).cop_deadline,
          "A timeout exactly at the IPBus endpoint still prevents the bus cycle from completing");
}

void test_native_invalid_service() {
  // Own LDAA#1; STAA COPCTL; LDAA#F0; STAA ARMCOP; NOP, with literal generic CPU opcodes.
  constexpr std::array<std::uint8_t, 11> code{
      0x86, 1, 0x7a, 0, 0x3c, 0x86, 0xf0, 0x7a, 0, 0x3f, 0xa7};
  const auto nvm = blofeld::test::checkpoint_image(code);
  McuMachine machine(nvm, BLOFELD_DEFAULT_SLA, contract());
  blofeld::test::drive_interrupts(machine.bus());
  for (unsigned n = 0; n < 3; ++n) { machine.cpu().step(); }
  require(machine.bus().cop_state().control == 1 &&
              machine.bus().cop_state().reset == McuCopReset::none &&
              machine.cpu().state().pc == blofeld::test::entry + 7,
          "Actual native instructions arm COP and prepare the invalid service byte");
  const auto fault = error([&] { machine.cpu().step(); });
  require(
      fault.contains("MCU write at 0x003f") && fault.contains("pc=0x8007") &&
          fault.contains("invalid ARMCOP service") && machine.timing_faulted() &&
          machine.bus().cop_state().reset == McuCopReset::invalid_service &&
          machine.cpu().state().pc == blofeld::test::entry + 7,
      "The original kind of invalid kick latches reset and stops before later native instructions");
  const auto image = machine.bus().power_off_flash_image();
  require(image.bytes == nvm.bytes && image.known == nvm.known && image.fresh == nvm.fresh,
          "Requesting reset alone does not invent a reset vector, erase NVM or change history");
  const auto ticks = machine.run_clock().elapsed_ticks();
  static_cast<void>(error([&] { machine.cpu().step(); }));
  require(machine.run_clock().elapsed_ticks() == ticks,
          "An unhandled reset request cannot accrue hidden CPU time");
}

void test_wait_gate_checkpoint_and_wake() {
  constexpr std::array<std::uint8_t, 2> code{0x3e, 0xa7}; // Own WAI; NOP.
  McuMachine machine(blofeld::test::checkpoint_image(code), BLOFELD_DEFAULT_SLA, contract());
  blofeld::test::drive_interrupts(machine.bus());
  machine.cpu().set_state(
      {.pc = blofeld::test::entry, .sp = blofeld::test::stack, .ccr = irq_unmasked});
  machine.bus().write(hc12::clock_select, 1);
  machine.bus().write(control_address, 1);
  machine.cpu().step();
  machine.bus().write(service_address, arm);
  const auto frozen = machine.checkpoint();
  require(machine.bus().cop_state().phase == 14 && machine.bus().cop_state().armed,
          "The seven native WAI-entry cycles still clock COP before sleeping");
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    machine.restore(frozen);
    require(machine.advance_wait(held_wait) == held_wait && machine.bus().cop_state().phase == 14 &&
                machine.bus().cop_state().armed && !machine.timing_faulted(),
            "COPWAI freezes the exact partial counter/service sequence and checkpoint replay");
    machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 2});
    const auto wake = machine.cpu().step();
    require(wake.interrupt_vector == blofeld::test::xirq_vector && wake.stack_reused &&
                machine.bus().cop_state().phase == wake_phase,
            "The six native wake cycles resume COP on actual OSCCLK edges");
    blofeld::test::drive_interrupts(machine.bus());
    machine.cpu().step();
    require(machine.cpu().state().pc == blofeld::test::entry + 1 &&
                machine.bus().cop_state().phase == returned_phase,
            "The eight native RTI cycles preserve and extend the same watchdog period");
  }
}

void test_wait_timeout_is_not_a_reset_stub() {
  constexpr std::array<std::uint8_t, 2> code{0x3e, 0xa7};
  McuMachine machine(blofeld::test::checkpoint_image(code), BLOFELD_DEFAULT_SLA, contract());
  blofeld::test::drive_interrupts(machine.bus());
  machine.cpu().set_state(
      {.pc = blofeld::test::entry, .sp = blofeld::test::stack, .ccr = irq_unmasked});
  machine.bus().write(hc12::clock_select, 0);
  machine.bus().write(control_address, 1);
  machine.cpu().step();
  const auto before = machine.checkpoint();
  const auto fault = error([&] { machine.advance_wait(held_wait); });
  require(fault.contains("COP system reset (timeout)") && machine.timing_faulted() &&
              machine.idle_bus_cycles() == before_expiry_idle &&
              machine.run_clock().bus_cycles() == expired_bus_cycles &&
              machine.bus().cop_state().phase == first_period &&
              machine.bus().cop_state().reset == McuCopReset::timeout,
          "Ungated WAI expires at the literal edge and preserves the failed-cycle prefix, not a "
          "fake boot");
  static_cast<void>(error([&] { machine.advance_wait(1); }));
  machine.restore(before);
  require(machine.bus().cop_state().phase == 14 &&
              machine.bus().cop_state().reset == McuCopReset::none,
          "Only trusted coordinated restore recovers the pre-event native owner state");
}
}

int main() {
  try {
    test_register_domain_and_lock();
    test_rate_deadlines();
    test_service_sequences();
    test_window_edges_and_invalid_bytes();
    test_clock_deadline_and_relocation();
    test_fractional_phy_edges();
    test_pure_deadline_offers();
    test_native_invalid_service();
    test_wait_gate_checkpoint_and_wake();
    test_wait_timeout_is_not_a_reset_stub();
    std::cout << "Normal COP registers, seven rates, service/window edges, copies and native WAI "
                 "requests verified.\n";
    return 0;
  } catch (const std::exception &fault) {
    std::cerr << fault.what() << '\n';
    return 1;
  }
}
