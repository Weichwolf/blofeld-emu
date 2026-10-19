// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/mcu_sci.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "mcu_sci_reference_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace {
using namespace blofeld::test;
namespace reg = blofeld::hc12;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned divisor = 60;
constexpr std::uint64_t bit_period = std::uint64_t{16} * divisor;
constexpr unsigned last_delay = 360;
constexpr unsigned serial_vector = 0xffd6;
constexpr unsigned note_on = 0x90;
constexpr unsigned stop_decision = 10200; // 16 mark + 154 receiver ticks, each 60 periods.
constexpr unsigned reset_and_wait = 12;
constexpr unsigned unmasked = 0xc0;
constexpr unsigned wait_stack = stack - 9;
constexpr unsigned pause_cycles = 17;
constexpr unsigned timer_counter = 0x44;
constexpr unsigned native_control2 = 0x2c;
constexpr unsigned paused_control1 = 0x48;
constexpr unsigned initial_ticks = 64;
constexpr blofeld::CpuState foreground{
    .pc = entry, .d = 0x1234, .x = 0x4567, .y = 0x6789, .sp = stack, .ccr = unmasked};

blofeld::McuFlashImage program() {
  constexpr std::array<std::uint8_t, 3> main{0x3e, 0xa7, 0x3e}; // Own WAI; NOP; WAI.
  auto result = checkpoint_image(main);
  // Own LDAB SCISR1; LDAA SCIDRL; STAA $1200; RTI. Costs 3+3+3+8.
  constexpr std::array<std::uint8_t, 10> isr{0xf6, 0, 0xcc, 0xb6, 0, 0xcf, 0x7a, 0x12, 0, 0x0b};
  for (unsigned index = 0; index < isr.size(); ++index) {
    result.bytes.at(handler - entry + index) = isr[index];
  }
  result.bytes.at(serial_vector - entry) = handler >> 8U;
  result.bytes.at(serial_vector - entry + 1) = 0;
  return result;
}

blofeld::McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads};
}

void configure(blofeld::McuMachine &machine) {
  auto &bus = machine.bus();
  drive_interrupts(bus);
  bus.write(reg::clock_select, 0);
  bus.write(reg::usb_control, 0x40);
  bus.observe_phy({.edges = 15});
  bus.write(reg::clock_select, 0x80);
  machine.cpu().set_state(foreground);
  machine.cpu().step();
  require(machine.cpu().waiting(), "Own program enters native WAI before serial clocks start");
  bus.write(reg::sci_baud_low, divisor);
  bus.write(reg::sci_control1, 8);
  bus.write(reg::sci_control2, native_control2);
  bus.write(reg::tscr1, 0x80);
}

void attach_wire(blofeld::McuMachine &machine) {
  const auto drive = [&machine] {
    const auto elapsed = machine.run_clock().bus_cycles() - reset_and_wait;
    constexpr unsigned mark_end = 16 * divisor;
    constexpr unsigned start_end = 32 * divisor;
    constexpr unsigned data_end = 160 * divisor;
    bool high = elapsed < mark_end || elapsed >= data_end;
    if (elapsed >= start_end && elapsed < data_end) {
      const auto bit = static_cast<unsigned>((elapsed - start_end) / bit_period);
      high = ((note_on >> bit) & 1U) != 0;
    }
    machine.bus().update_pin_drive(
        blofeld::McuPort::r, 4, {.mask = 4, .levels = static_cast<std::uint8_t>(high ? 4 : 0)});
  };
  machine.set_wait_observer([drive](auto /*cycle*/) { drive(); });
  machine.set_cycle_observer([drive](auto /*cycle*/) { drive(); });
}

void check_frame(blofeld::McuMachine &machine) {
  constexpr std::array<unsigned, 9> expected{unmasked, 0x34, 0x12, 0x45, 0x67, 0x67, 0x89, 0x80, 1};
  for (unsigned index = 0; index < expected.size(); ++index) {
    require(machine.bus().read(static_cast<std::uint16_t>(wait_stack + index)) == expected[index],
            "Each independent NXP frame byte survives wait and SCI interrupt entry");
  }
}

void wake_and_return(blofeld::McuMachine &machine, const unsigned elapsed) {
  const auto request = machine.cpu().pending_interrupt();
  require(
      request && request->vector == serial_vector &&
          machine.run_clock().bus_cycles() == reset_and_wait + elapsed &&
          machine.run_clock().elapsed_ticks() == initial_ticks + (2 * elapsed) &&
          machine.bus().read_memory({.address = timer_counter, .size = 2}) == elapsed,
      "The exact declared transfer edge raises native SCI with independently advancing TIM/OSC");
  check_frame(machine);
  const auto entered = machine.cpu().step();
  require(entered.interrupt_vector == serial_vector && entered.stack_reused &&
              !entered.instruction_executed && machine.cpu().state().pc == handler,
          "Native SCI wake reuses the pre-existing WAI frame through fVfPPP");
  check_frame(machine);
  for (unsigned instruction = 0; instruction < 4; ++instruction) { machine.cpu().step(); }
  const auto restored = machine.cpu().state();
  require(
      restored.pc == entry + 1 && restored.sp == stack && restored.d == foreground.d &&
          restored.x == foreground.x && restored.y == foreground.y &&
          restored.ccr == foreground.ccr && machine.bus().read(ram) == note_on &&
          !machine.cpu().pending_interrupt(),
      "Native status/data acknowledgement, RAM store and RTI restore every foreground register");
  machine.cpu().step(); // Native foreground NOP.
  machine.cpu().step(); // Native foreground WAI.
  constexpr unsigned wake_handler_and_main = 6 + 3 + 3 + 3 + 8 + 1 + 7;
  require(
      machine.cpu().waiting() && machine.cpu().state().pc == entry + 3 &&
          machine.run_clock().bus_cycles() == reset_and_wait + elapsed + wake_handler_and_main &&
          machine.idle_bus_cycles() == elapsed,
      "Literal manual instruction costs reach the next WAI without replacing CPU/peripheral state");
}

void test_phase(const blofeld::McuCheckpoint &baseline,
                blofeld::McuMachine &machine,
                const unsigned delay) {
  machine.restore(baseline);
  machine.set_sci_receive_reference(
      blofeld::SciReceiveReference{.baud_divisor = divisor, .transfer_delay_bus_cycles = delay});
  attach_wire(machine);
  if (delay == 0) {
    require(machine.advance_wait(stop_decision + last_delay) == stop_decision,
            "Immediate reference transfer stops WAI exactly at the RT10 decision");
    wake_and_return(machine, stop_decision);
    return;
  }
  require(machine.advance_wait(stop_decision) == stop_decision &&
              !machine.cpu().pending_interrupt(),
          "A positive declared delay retains a pending transfer beyond RT10");
  machine.bus().write(reg::sci_control1, paused_control1);
  require(machine.advance_wait(pause_cycles) == pause_cycles && !machine.cpu().pending_interrupt(),
          "SCISWAI freezes a pending transfer deadline independently of TIM/OSC");
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    attach_wire(machine);
    const auto reference = machine.sci_receive_reference();
    require(reference && reference->transfer_delay_bus_cycles == delay,
            "Trusted restore retains the explicit timing scenario and pending deadline");
    machine.bus().write(reg::sci_control1, 8);
    require(machine.advance_wait(last_delay + 1) == delay,
            "A restored pending transfer consumes exactly its remaining enabled IPBus periods");
    wake_and_return(machine, stop_decision + pause_cycles + delay);
  }
}
}

void test_owned_sci_references() {
  blofeld::McuMachine machine(program(), BLOFELD_DEFAULT_SLA, contract());
  configure(machine);
  require(!machine.sci_receive_reference(), "Owned MCU default has no selected SCI hypothesis");
  const auto baseline = machine.checkpoint();
  attach_wire(machine);
  reject([&] { machine.advance_wait(stop_decision + 1); },
         "Unqualified default still exposes the transfer window during continuous IRQ selection");
  require(machine.timing_faulted(), "Unknown SCI transfer timing quarantines the owned clock");
  reject(
      [&] {
        machine.set_sci_receive_reference(blofeld::SciReceiveReference{.baud_divisor = divisor});
      },
      "A reference cannot repair an already failed run without trusted restore");
  for (const unsigned delay :
       {0U, 1U, divisor - 1, divisor, divisor + 1, last_delay - 1, last_delay}) {
    test_phase(baseline, machine, delay);
  }
  machine.restore(baseline);
  require(!machine.sci_receive_reference(),
          "Restoring the unqualified checkpoint drops the hypothesis");
  machine.set_wait_observer([&machine](auto /*cycle*/) {
    reject([&] { machine.set_sci_receive_reference(std::nullopt); },
           "SCI reference reconfiguration cannot reenter active native WAI");
    machine.bus().update_pin_drive(blofeld::McuPort::r, 4, {.mask = 4, .levels = 4});
  });
  require(machine.advance_wait(1) == 1 && !machine.timing_faulted(),
          "Caught reference reconfiguration guards leave a healthy WAI boundary");
}
