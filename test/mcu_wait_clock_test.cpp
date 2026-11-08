// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "cpu12/memory.hpp"
#include "isa.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld::test;
namespace isa = cpu12::isa;
using cpu12::Byte;
using isa::Opcode;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned wait_stack = stack - 9;
constexpr Byte unmasked = 0xc0, masked = 0xd0;
constexpr unsigned initial_d = 0x1234, initial_x = 0x4567, initial_y = 0x6789;
constexpr unsigned realtime_vector = 0xfff0, timer_vector = 0xffee;
constexpr unsigned reset_and_wait = 12;
constexpr Byte stop_in_wait = 0x40, enabled = 0x80;
constexpr unsigned counter = 0x44, data_pattern = 0x55, receive_pad = 4;
constexpr unsigned frame_cycles = 192, short_idle = 17, remaining_idle = 2541;
constexpr unsigned idle_limit = 5000, wake_and_handler_cycles = 18, final_timer = 2583;
constexpr unsigned receive_ready = 0xe0, receive_idle = 0xc0;
constexpr unsigned external_idle_bound = 100, relocated_clock_select = 0x1039;

blofeld::McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads};
}

blofeld::McuFlashImage image() {
  auto result = checkpoint_image(isa::program(isa::wai, isa::nop));
  // Acknowledge RTIF and return: 1 + 3 + 8 nominal cycles.
  constexpr auto isr = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                    std::array<Byte, 1>{enabled},
                                    isa::opcode<Opcode::staa_extended>(),
                                    std::array<Byte, 2>{0, blofeld::hc12::crg_flags},
                                    isa::rti);
  for (unsigned index = 0; index < isr.size(); ++index) {
    result.bytes.at(handler - entry + index) = isr[index];
  }
  for (const unsigned vector : {realtime_vector, timer_vector}) {
    result.bytes.at(vector - entry) = handler >> 8U;
    result.bytes.at(vector - entry + 1) = 0;
  }
  return result;
}

struct Fixture {
  blofeld::McuMachine machine{image(), BLOFELD_DEFAULT_SLA, contract()};

  explicit Fixture(const Byte flags = unmasked) {
    drive_interrupts(machine.bus());
    machine.cpu().set_state(
        {.pc = entry, .d = initial_d, .x = initial_x, .y = initial_y, .sp = stack, .ccr = flags});
  }

  void enter() {
    machine.cpu().step();
    const auto state = machine.cpu().state();
    require(machine.cpu().waiting() && state.pc == entry + 1 && state.sp == wait_stack &&
                state.d == initial_d && state.x == initial_x && state.y == initial_y &&
                machine.run_clock().bus_cycles() == reset_and_wait &&
                machine.idle_bus_cycles() == 0,
            "WAI stacks its frame in seven cycles after the five reset cycles");
  }
};

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected owned wait rejection");
}

struct Counts {
  std::uint64_t cycles, idle, ticks, oscillator;
};

void check_counts(const blofeld::McuMachine &machine, const Counts expected) {
  require(!machine.timing_faulted() && machine.run_clock().bus_cycles() == expected.cycles &&
              machine.idle_bus_cycles() == expected.idle &&
              machine.run_clock().elapsed_ticks() == expected.ticks &&
              machine.run_clock().oscillator_cycles() == expected.oscillator,
          "CPU/idle periods and fractional oscillator phase match independent literal counts");
}

void check_frame(blofeld::McuMachine &machine) {
  // S12CPUV2 CCR/B/A/X/Y/next-PC layout, independent of the push/pop implementation.
  constexpr std::array<unsigned, 9> frame{unmasked, 0x34, 0x12, 0x45, 0x67, 0x67, 0x89, 0x80, 1};
  for (unsigned index = 0; index < frame.size(); ++index) {
    require(machine.bus().read(static_cast<std::uint16_t>(wait_stack + index)) == frame[index],
            "WAI stores every literal frame byte in its specified order");
  }
}

void fractional_rti_wake() {
  Fixture fixture;
  auto &machine = fixture.machine;
  auto &bus = machine.bus();
  bus.write(blofeld::hc12::clock_select, 0);
  bus.write(blofeld::hc12::usb_control, stop_in_wait);
  bus.observe_phy({.edges = 15});
  bus.write(blofeld::hc12::clock_select, enabled);
  bus.write(blofeld::hc12::tscr1, enabled);
  fixture.enter();
  check_frame(machine);
  check_counts(machine, {.cycles = 12, .idle = 0, .ticks = 64, .oscillator = 12});
  bus.write(blofeld::hc12::rti_control, 0x10); // 1024 oscillator edges.
  bus.write(blofeld::hc12::crg_interrupts, enabled);
  require(machine.advance_wait(short_idle) == short_idle, "A short idle slice respects its bound");
  constexpr Counts partial{.cycles = 29, .idle = 17, .ticks = 98, .oscillator = 19};
  check_counts(machine, partial);
  unsigned observed = 0;
  unsigned cpu_cycles = 0;
  machine.set_span_observer([&](const blofeld::McuSpan span) {
    if (!span.waiting) {
      cpu_cycles += span.cycles;
      return;
    }
    require(span.context.pc == entry + 1 && span.context.ccr == unmasked,
            "Completed idle spans retain the actual WAI context");
    observed += span.cycles;
  });
  require(machine.advance_wait(idle_limit) == remaining_idle && observed == remaining_idle &&
              cpu_cycles == 0,
          "The fractional oscillator phase reaches RTI after 2558 idle periods");
  constexpr Counts pending{.cycles = 2570, .idle = 2558, .ticks = 5180, .oscillator = 1036};
  check_counts(machine, pending);
  const auto interrupt = machine.cpu().pending_interrupt();
  require(machine.advance_wait(idle_limit) == 0 && observed == remaining_idle && interrupt &&
              interrupt->vector == realtime_vector,
          "A pending RTI interrupt adds no idle period");
  const auto entered = machine.cpu().step();
  require(!entered.instruction_executed && entered.interrupt_vector == realtime_vector &&
              entered.stack_reused && machine.cpu().state().sp == wait_stack && cpu_cycles == 6,
          "WAI wake reuses its existing frame and charges six native cycles");
  check_frame(machine);
  for (unsigned instruction = 0; instruction < 3; ++instruction) { machine.cpu().step(); }
  const auto state = machine.cpu().state();
  require(!machine.cpu().waiting() && !machine.cpu().pending_interrupt() && state.pc == entry + 1 &&
              state.sp == stack && state.ccr == unmasked && state.d == initial_d &&
              state.x == initial_x && state.y == initial_y &&
              cpu_cycles == wake_and_handler_cycles &&
              bus.read_memory({.address = counter, .size = 2}) == final_timer,
          "The real handler clears RTIF and restores the complete foreground register set");
  constexpr Counts returned{.cycles = 2588, .idle = 2558, .ticks = 5216, .oscillator = 1043};
  check_counts(machine, returned);
  machine.validate_run_boundary();
}

void configure_sci(blofeld::McuBus &bus, const Byte wait_gate) {
  bus.write(blofeld::hc12::sci_control1, wait_gate);
  bus.write(blofeld::hc12::sci_baud_high, 0);
  bus.write(blofeld::hc12::sci_baud_low, 1);
  bus.write(blofeld::hc12::sci_control2, 4);
}

void drive_frame(blofeld::McuBus &bus, const unsigned elapsed) {
  constexpr unsigned bit_cycles = 16;
  constexpr unsigned data_begin = 32;
  constexpr unsigned data_end = 160;
  bool high = true;
  if (elapsed >= bit_cycles && elapsed < data_begin) { high = false; }
  if (elapsed >= data_begin && elapsed < data_end) {
    high = ((data_pattern >> ((elapsed - data_begin) / bit_cycles)) & 1U) != 0;
  }
  bus.set_pin_drive(blofeld::McuPort::r,
                    {.mask = receive_pad, .levels = static_cast<Byte>(high ? receive_pad : 0U)});
}

void sci_timer_gates() {
  for (const Byte sci_gate : {Byte{0}, stop_in_wait}) {
    for (const Byte timer_gate : {Byte{0}, stop_in_wait}) {
      Fixture fixture(masked);
      auto &machine = fixture.machine;
      fixture.enter();
      configure_sci(machine.bus(), sci_gate);
      machine.bus().write(blofeld::hc12::tscr1, static_cast<Byte>(enabled | timer_gate));
      machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
      machine.set_timer_inputs(1);
      if (timer_gate != 0) { machine.bus().set_pin_drive(blofeld::McuPort::t, {}); }
      // The caller supplies actual external edges before advancing their interval.
      for (unsigned cycle = 0; cycle < frame_cycles; ++cycle) {
        if (sci_gate == 0) { drive_frame(machine.bus(), cycle); }
        require(machine.advance_wait(1) == 1, "Masked WAI advances exactly the supplied interval");
      }
      require(machine.bus().read_memory({.address = counter, .size = 2}) ==
                      (timer_gate == 0 ? frame_cycles : 0) &&
                  machine.bus().read(blofeld::hc12::sci_status1) ==
                      (sci_gate == 0 ? receive_ready : receive_idle),
              "TSWAI and SCISWAI independently gate the timer and NRZ receiver");
      if (sci_gate == 0) {
        require(machine.bus().read(blofeld::hc12::sci_data_low) == data_pattern,
                "The enabled SCI receives the independently supplied byte while WAI holds");
      }
      constexpr Counts received{.cycles = 204, .idle = 192, .ticks = 2040, .oscillator = 408};
      check_counts(machine, received);
    }
  }
}

void paused_sci_frame() {
  Fixture fixture(masked);
  auto &machine = fixture.machine;
  fixture.enter();
  configure_sci(machine.bus(), 0);
  constexpr unsigned middle = 96;
  constexpr unsigned pause = 257;
  for (unsigned cycle = 0; cycle < frame_cycles; ++cycle) {
    if (cycle == middle) {
      machine.bus().write(blofeld::hc12::sci_control1, stop_in_wait);
      machine.bus().set_pin_drive(blofeld::McuPort::r, {});
      constexpr Byte changed_format = 0x42;
      static_cast<void>(
          rejected([&] { machine.bus().write(blofeld::hc12::sci_control1, changed_format); }));
      require(machine.advance_wait(pause) == pause &&
                  machine.bus().read(blofeld::hc12::sci_status1) == receive_idle,
              "A gated half-frame retains its state without sampling unknown RX");
      machine.bus().write(blofeld::hc12::sci_control1, 0);
    }
    drive_frame(machine.bus(), cycle);
    require(machine.advance_wait(1) == 1,
            "The external source resumes the actual retained half-frame");
  }
  require(machine.bus().read(blofeld::hc12::sci_status1) == receive_ready &&
              machine.bus().read(blofeld::hc12::sci_data_low) == data_pattern,
          "Releasing SCISWAI resumes the receiver and preserves the complete byte");
  require(machine.idle_bus_cycles() == frame_cycles + pause,
          "Paused time remains actual device time");
  machine.validate_run_boundary();
}

void external_and_capture_wake() {
  Fixture fixture;
  auto &machine = fixture.machine;
  fixture.enter();
  machine.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 1});
  require(machine.advance_wait(external_idle_bound) == 0,
          "A pending external IRQ adds no idle time");
  const auto entered = machine.cpu().step();
  require(entered.interrupt_vector == irq_vector && entered.stack_reused &&
              !entered.instruction_executed && machine.cpu().state().sp == wait_stack,
          "The native IRQ wakes using the existing literal frame");
  check_frame(machine);
  constexpr Counts awake{.cycles = 18, .idle = 0, .ticks = 180, .oscillator = 36};
  check_counts(machine, awake);

  Fixture capture;
  auto &captured = capture.machine;
  capture.enter();
  captured.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
  captured.set_timer_inputs(1);
  constexpr unsigned capture_edges = 0x4b;
  constexpr unsigned timer_interrupts = 0x4c;
  constexpr unsigned capture_value = 0x50;
  captured.bus().write(capture_edges, 1);
  captured.bus().write(timer_interrupts, 1);
  captured.bus().write(blofeld::hc12::tscr1, enabled);
  require(captured.advance_wait(1) == 1, "The timer reaches literal counter one");
  captured.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1, .levels = 1});
  require(captured.advance_wait(external_idle_bound) == 0 &&
              captured.bus().read_memory({.address = capture_value, .size = 2}) == 1 &&
              captured.bus().read_memory({.address = counter, .size = 2}) == 1,
          "Input capture requests its IRQ at the current boundary before another idle cycle");
  const auto pending = captured.cpu().pending_interrupt();
  require(pending && pending->vector == timer_vector, "The actual capture flag selects its vector");
  constexpr Counts captured_counts{.cycles = 13, .idle = 1, .ticks = 130, .oscillator = 26};
  check_counts(captured, captured_counts);
}

void unknown_inputs_and_clocks() {
  for (unsigned kind = 0; kind < 3; ++kind) {
    Fixture fixture;
    auto &machine = fixture.machine;
    fixture.enter();
    if (kind == 0) { configure_sci(machine.bus(), 0); }
    if (kind == 1) {
      machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
      machine.set_timer_inputs(1);
      machine.bus().set_pin_drive(blofeld::McuPort::t, {});
    }
    if (kind == 2) {
      machine.bus().write(blofeld::hc12::usb_control, stop_in_wait);
      machine.bus().write(blofeld::hc12::clock_select, enabled);
    }
    const auto error = rejected([&] { machine.advance_wait(1); });
    require(error.contains("pc=0x8001") && machine.timing_faulted() &&
                machine.run_clock().bus_cycles() == reset_and_wait &&
                machine.idle_bus_cycles() == 0,
            "Unknown SCI/timer inputs and unqualified PHY fail before inventing a period");
    static_cast<void>(rejected([&] { machine.cpu().step(); }));
  }
  for (const Byte bit : {Byte{2}, Byte{4}, Byte{0x10}, Byte{0x20}}) {
    Fixture fixture;
    fixture.enter();
    fixture.machine.bus().write(blofeld::hc12::clock_select, bit);
    const auto error = rejected([&] { fixture.machine.advance_wait(1); });
    require(error.contains("wait clock at 0x0039") && error.contains("pc=0x8001") &&
                fixture.machine.timing_faulted() &&
                fixture.machine.run_clock().bus_cycles() == reset_and_wait,
            "Unsupported wait/PLL gates fail explicitly with actual context");
  }
}

void invalid_boundaries() {
  Fixture fixture;
  static_cast<void>(rejected([&] { fixture.machine.advance_wait(1); }));
  require(!fixture.machine.timing_faulted(), "A non-WAI rejection has no execution effects");
  fixture.enter();
  require(fixture.machine.advance_wait(0) == 0, "An empty valid slice adds no time");
  fixture.machine.bus().write(blofeld::hc12::initrg, 0x10);
  fixture.machine.bus().retire_instruction();
  fixture.machine.bus().write(relocated_clock_select, 2);
  require(rejected([&] { fixture.machine.advance_wait(1); }).contains("0x1039"),
          "Clock faults report the relocated live register address");
  blofeld::McuMachine untimed(image(), BLOFELD_DEFAULT_SLA);
  static_cast<void>(rejected([&] { untimed.advance_wait(1); }));
}
}

int main() {
  try {
    fractional_rti_wake();
    sci_timer_gates();
    paused_sci_frame();
    external_and_capture_wake();
    unknown_inputs_and_clocks();
    invalid_boundaries();
    std::cout
        << "Native WAI spans, domain gates, literal phase/frame counts and IRQ wake passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
