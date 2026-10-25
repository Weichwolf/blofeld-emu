// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned wait_stack = stack - 9;
constexpr unsigned unmasked = 0xc0;
constexpr unsigned masked = 0xd0;
constexpr unsigned initial_d = 0x1234;
constexpr unsigned initial_x = 0x4567;
constexpr unsigned initial_y = 0x6789;
constexpr unsigned realtime_vector = 0xfff0;
constexpr unsigned serial_vector = 0xffd6;
constexpr unsigned timer_vector = 0xffee;
constexpr unsigned reset_and_wait = 12;
constexpr unsigned stop_in_wait = 0x40;
constexpr unsigned enabled = 0x80;
constexpr unsigned counter = 0x44;
constexpr unsigned data_pattern = 0x55;
constexpr unsigned receive_pad = 4;
constexpr unsigned frame_cycles = 192;
constexpr unsigned short_idle = 17;
constexpr unsigned remaining_idle = 2541;
constexpr unsigned idle_limit = 5000;
constexpr unsigned wake_and_handler_cycles = 18;
constexpr unsigned final_timer = 2583;
constexpr unsigned receive_ready = 0xe0;
constexpr unsigned receive_idle = 0xc0;
constexpr unsigned external_idle_bound = 100;
constexpr unsigned failed_idle_ticks = 140;
constexpr unsigned relocated_clock_select = 0x1039;

blofeld::McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads};
}

blofeld::McuFlashImage image() {
  constexpr std::array<std::uint8_t, 2> code{0x3e, 0xa7}; // Own WAI; NOP.
  auto result = checkpoint_image(code);
  // Own LDAA #$80; STAA $0037; RTI. Manual costs: 1+3+8 cycles.
  constexpr std::array<std::uint8_t, 6> isr{0x86, 0x80, 0x7a, 0, 0x37, 0x0b};
  for (unsigned index = 0; index < isr.size(); ++index) {
    result.bytes.at(handler - entry + index) = isr[index];
  }
  for (const unsigned vector : {realtime_vector, serial_vector, timer_vector}) {
    result.bytes.at(vector - entry) = handler >> 8U;
    result.bytes.at(vector - entry + 1) = 0;
  }
  return result;
}

struct Fixture {
  blofeld::McuMachine machine{image(), BLOFELD_DEFAULT_SLA, contract()};

  explicit Fixture(const std::uint8_t flags = unmasked) {
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
            "Native WAI stacks its frame in seven cycles after the five reset cycles");
  }
};

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected owned wait rejection");
}

struct Counts {
  std::uint64_t cycles;
  std::uint64_t idle;
  std::uint64_t ticks;
  std::uint64_t oscillator;
};

constexpr Counts phy_wait_counts{.cycles = 12, .idle = 0, .ticks = 64, .oscillator = 12};
constexpr Counts partial_wait_counts{.cycles = 29, .idle = 17, .ticks = 98, .oscillator = 19};
constexpr Counts pending_rti_counts{
    .cycles = 2570, .idle = 2558, .ticks = 5180, .oscillator = 1036};
constexpr Counts returned_rti_counts{
    .cycles = 2588, .idle = 2558, .ticks = 5216, .oscillator = 1043};
constexpr Counts serial_frame_counts{.cycles = 204, .idle = 192, .ticks = 2040, .oscillator = 408};
constexpr Counts external_wake_counts{.cycles = 18, .idle = 0, .ticks = 180, .oscillator = 36};
constexpr Counts one_idle_counts{.cycles = 13, .idle = 1, .ticks = 130, .oscillator = 26};
constexpr Counts recovered_idle_counts{.cycles = 15, .idle = 3, .ticks = 150, .oscillator = 30};

void check_counts(const blofeld::McuMachine &machine, const Counts expected) {
  require(!machine.timing_faulted() && machine.run_clock().bus_cycles() == expected.cycles &&
              machine.idle_bus_cycles() == expected.idle &&
              machine.run_clock().elapsed_ticks() == expected.ticks &&
              machine.run_clock().oscillator_cycles() == expected.oscillator,
          "CPU/idle IPBus periods and fractional OSC phase match literal independent counts");
}

void select_phy(blofeld::McuBus &bus) {
  bus.write(blofeld::hc12::clock_select, 0);
  bus.write(blofeld::hc12::usb_control, 0x40);
  bus.observe_phy({.edges = 15});
  bus.write(blofeld::hc12::clock_select, enabled);
}

void check_frame(blofeld::McuMachine &machine) {
  // S12CPUV2 independent CCR/B/A/X/Y/next-PC layout, not a push/pop round trip.
  constexpr std::array<unsigned, 9> frame{unmasked, 0x34, 0x12, 0x45, 0x67, 0x67, 0x89, 0x80, 1};
  for (unsigned index = 0; index < frame.size(); ++index) {
    require(machine.bus().read(static_cast<std::uint16_t>(wait_stack + index)) == frame[index],
            "Native WAI stores every literal NXP frame byte in its specified order");
  }
}

void test_fractional_rti_wake_and_replay() {
  Fixture fixture;
  auto &machine = fixture.machine;
  select_phy(machine.bus());
  machine.bus().write(blofeld::hc12::tscr1, enabled);
  fixture.enter();
  check_frame(machine);
  check_counts(machine, phy_wait_counts);
  machine.bus().write(blofeld::hc12::rti_control, 0x10); // 1024 OSC edges.
  machine.bus().write(blofeld::hc12::crg_interrupts, enabled);
  require(machine.advance_wait(short_idle) == short_idle,
          "A short idle slice stops at its explicit bound");
  check_counts(machine, partial_wait_counts);
  const auto frozen = machine.checkpoint();
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    machine.restore(frozen);
    unsigned observed = 0;
    unsigned cpu_cycles = 0;
    machine.set_wait_observer([&observed](const auto cycle) {
      require(cycle.context.pc == entry + 1 && cycle.context.ccr == unmasked,
              "Idle observations retain WAI context without inventing instruction slots");
      ++observed;
    });
    machine.set_cycle_observer([&cpu_cycles](auto /*cycle*/) { ++cpu_cycles; });
    require(machine.advance_wait(idle_limit) == remaining_idle && observed == remaining_idle &&
                cpu_cycles == 0,
            "The unfinished OSC fraction reaches RTI after exactly 2558 total idle periods");
    check_counts(machine, pending_rti_counts);
    const auto no_more_idle = machine.advance_wait(idle_limit);
    const auto pending = machine.cpu().pending_interrupt();
    require(no_more_idle == 0 && observed == remaining_idle && pending &&
                pending->vector == realtime_vector,
            "An already pending native interrupt adds no idle time or observation callback");
    const auto result = machine.cpu().step();
    require(!result.instruction_executed && result.interrupt_vector == realtime_vector &&
                result.stack_reused && machine.cpu().state().sp == wait_stack && cpu_cycles == 6,
            "Native WAI wake uses fVfPPP and the existing frame rather than a second push");
    check_frame(machine);
    for (unsigned instruction = 0; instruction < 3; ++instruction) { machine.cpu().step(); }
    const auto state = machine.cpu().state();
    require(!machine.cpu().waiting() && !machine.cpu().pending_interrupt() &&
                state.pc == entry + 1 && state.sp == stack && state.ccr == unmasked &&
                state.d == initial_d && state.x == initial_x && state.y == initial_y &&
                cpu_cycles == wake_and_handler_cycles &&
                machine.bus().read_memory({.address = counter, .size = 2}) == final_timer,
            "The native handler acknowledges RTIF and RTI restores every foreground register");
    check_counts(machine, returned_rti_counts);
    const auto completed = machine.checkpoint();
    machine.cpu().step();
    machine.restore(completed);
    check_counts(machine, returned_rti_counts);
  }
}

void configure_sci(blofeld::McuBus &bus, const unsigned wait_gate) {
  bus.write(blofeld::hc12::sci_control1, static_cast<std::uint8_t>(wait_gate));
  bus.write(blofeld::hc12::sci_baud_high, 0);
  bus.write(blofeld::hc12::sci_baud_low, 1);
  bus.write(blofeld::hc12::sci_control2, 4);
}

void drive_frame(blofeld::McuBus &bus, const std::uint64_t elapsed) {
  bool high = true;
  constexpr unsigned bit_cycles = 16;
  constexpr unsigned data_begin = 32;
  constexpr unsigned data_end = 160;
  if (elapsed >= bit_cycles && elapsed < data_begin) { high = false; }
  if (elapsed >= data_begin && elapsed < data_end) {
    const auto bit = static_cast<unsigned>((elapsed - data_begin) / bit_cycles);
    high = ((data_pattern >> bit) & 1U) != 0;
  }
  bus.set_pin_drive(
      blofeld::McuPort::r,
      {.mask = receive_pad, .levels = static_cast<std::uint8_t>(high ? receive_pad : 0)});
}

void test_sci_timer_wait_gate_matrix() {
  for (const unsigned sci_gate : {0U, stop_in_wait}) {
    for (const unsigned timer_gate : {0U, stop_in_wait}) {
      Fixture fixture(masked);
      auto &machine = fixture.machine;
      fixture.enter();
      configure_sci(machine.bus(), sci_gate);
      machine.bus().write(blofeld::hc12::tscr1, static_cast<std::uint8_t>(enabled | timer_gate));
      machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
      machine.set_timer_inputs(1);
      if (timer_gate != 0) { machine.bus().set_pin_drive(blofeld::McuPort::t, {}); }
      machine.set_wait_observer([&machine, sci_gate](auto /*cycle*/) {
        if (sci_gate == 0) { drive_frame(machine.bus(), machine.idle_bus_cycles()); }
      });
      require(machine.advance_wait(frame_cycles) == frame_cycles,
              "Masked WAI remains bounded while independently enabled domains run");
      const auto timer = machine.bus().read_memory({.address = counter, .size = 2});
      const auto status = machine.bus().read(blofeld::hc12::sci_status1);
      require(timer == (timer_gate == 0 ? frame_cycles : 0) &&
                  status == (sci_gate == 0 ? receive_ready : receive_idle),
              "TSWAI and SCISWAI independently gate counters and literal NRZ reception");
      if (sci_gate == 0) {
        require(machine.bus().read(blofeld::hc12::sci_data_low) == data_pattern,
                "The running SCI receives the independent on-wire byte while the CPU waits");
      }
      check_counts(machine, serial_frame_counts);
      const auto frozen = machine.checkpoint();
      machine.restore(frozen);
      check_counts(machine, serial_frame_counts);
    }
  }
}

void test_inflight_sci_pause_and_resume() {
  constexpr unsigned prefix_nops = 75;
  constexpr unsigned before_pause_cycles = 91;
  constexpr unsigned frozen_idle = 257;
  constexpr unsigned further_idle = 43;
  constexpr unsigned retained_idle = 300;
  constexpr unsigned reset_cycles = 5;
  constexpr unsigned changed_format = 0x42;
  constexpr Counts final_counts{.cycles = 497, .idle = 300, .ticks = 4970, .oscillator = 994};
  auto program = image();
  constexpr unsigned nop_opcode = 0xa7;
  program.bytes[0] = nop_opcode;
  // Own LDAA #SCISWAI; STAA $00ca; WAI, after 75 native NOPs.
  constexpr std::array<std::uint8_t, 6> gate_then_wait{0x86, 0x40, 0x7a, 0, 0xca, 0x3e};
  for (unsigned index = 0; index < gate_then_wait.size(); ++index) {
    program.bytes.at(prefix_nops + index) = gate_then_wait[index];
  }
  blofeld::McuMachine machine(program, BLOFELD_DEFAULT_SLA, contract());
  drive_interrupts(machine.bus());
  machine.cpu().set_state({.pc = entry, .d = initial_d, .sp = stack, .ccr = unmasked});
  configure_sci(machine.bus(), 0);
  const auto attach_wire = [&machine] {
    machine.set_cycle_observer([&machine](auto /*cycle*/) {
      const auto active = machine.run_clock().bus_cycles() - machine.idle_bus_cycles();
      drive_frame(machine.bus(), active - reset_cycles);
    });
  };
  attach_wire();
  while (!machine.cpu().waiting() && machine.run_clock().bus_cycles() < frame_cycles) {
    machine.cpu().step();
  }
  require(machine.cpu().waiting() && machine.run_clock().bus_cycles() == before_pause_cycles &&
              machine.bus().read(blofeld::hc12::sci_control1) == stop_in_wait &&
              machine.bus().read(blofeld::hc12::sci_status1) == receive_idle,
          "Native SCISWAI write during a receive changes no frame format and enters WAI mid-byte");
  static_cast<void>(
      rejected([&] { machine.bus().write(blofeld::hc12::sci_control1, changed_format); }));
  machine.bus().set_pin_drive(blofeld::McuPort::r, {});
  require(machine.advance_wait(frozen_idle) == frozen_idle &&
              machine.bus().read(blofeld::hc12::sci_status1) == receive_idle,
          "A gated in-flight receiver retains its sample/bit state without reading unknown RX");
  const auto frozen = machine.checkpoint();
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    machine.restore(frozen);
    attach_wire();
    require(machine.advance_wait(further_idle) == further_idle &&
                machine.idle_bus_cycles() == retained_idle,
            "Trusted wait restore preserves paused SCI state and independent idle accounting");
    machine.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 1});
    const auto entered = machine.cpu().step();
    require(entered.interrupt_vector == irq_vector && entered.stack_reused,
            "A real external IRQ resumes the paused SCI through native WAI wake");
    drive_interrupts(machine.bus());
    for (unsigned instruction = 0; instruction < 3; ++instruction) { machine.cpu().step(); }
    while (machine.run_clock().bus_cycles() - machine.idle_bus_cycles() <
           reset_cycles + frame_cycles) {
      machine.cpu().step();
    }
    require(machine.bus().read(blofeld::hc12::sci_status1) == receive_ready &&
                machine.bus().read(blofeld::hc12::sci_data_low) == data_pattern,
            "The retained half-frame resumes after wake and receives the complete literal byte");
    check_counts(machine, final_counts);
  }
}

void test_external_interrupt_at_idle_boundary() {
  for (const bool callback : {false, true}) {
    Fixture fixture;
    auto &machine = fixture.machine;
    fixture.enter();
    unsigned observed = 0;
    const auto assert_irq = [&machine] {
      machine.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 1});
    };
    machine.set_wait_observer([&observed, &assert_irq](auto /*cycle*/) {
      ++observed;
      assert_irq();
    });
    if (!callback) { assert_irq(); }
    require(machine.advance_wait(external_idle_bound) == 0 && observed == (callback ? 1U : 0U),
            "An interrupt present at either input boundary does not overshoot by a bus period");
    const auto entered = machine.cpu().step();
    require(entered.interrupt_vector == irq_vector && entered.stack_reused &&
                !entered.instruction_executed && machine.cpu().state().sp == wait_stack,
            "The native external IRQ wakes WAI using the existing literal stack frame");
    check_counts(machine, external_wake_counts);
    check_frame(machine);
  }
}

void test_capture_interrupt_at_idle_boundary() {
  constexpr unsigned capture_edges = 0x4b;
  constexpr unsigned timer_interrupts = 0x4c;
  constexpr unsigned capture_value = 0x50;
  Fixture fixture;
  auto &machine = fixture.machine;
  fixture.enter();
  machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
  machine.set_timer_inputs(1);
  machine.bus().write(capture_edges, 1);
  machine.bus().write(timer_interrupts, 1);
  machine.bus().write(blofeld::hc12::tscr1, enabled);
  require(machine.advance_wait(1) == 1, "The timer advances to literal counter one");
  machine.set_wait_observer([&machine](auto /*cycle*/) {
    machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1, .levels = 1});
  });
  require(machine.advance_wait(external_idle_bound) == 0 &&
              machine.bus().read_memory({.address = capture_value, .size = 2}) == 1 &&
              machine.bus().read_memory({.address = counter, .size = 2}) == 1,
          "A sampled capture at the current boundary requests IRQ before another idle period");
  const auto pending = machine.cpu().pending_interrupt();
  require(pending && pending->vector == timer_vector,
          "The actual timer flag/enable route requests its native vector");
  check_counts(machine, one_idle_counts);
}

void test_active_guards_and_callback_lifetime() {
  Fixture fixture;
  auto &machine = fixture.machine;
  fixture.enter();
  const auto frozen = machine.checkpoint();
  auto &held_cpu = machine.cpu();
  unsigned observed = 0;
  unsigned committed = 0;
  machine.set_wait_period_observer([&](const auto period) {
    require(period.context.pc == entry + 1 && period.context.ccr == unmasked,
            "Committed idle periods retain actual WAI context without inventing CPU slots");
    static_cast<void>(rejected([&] { machine.set_wait_period_observer({}); }));
    static_cast<void>(rejected([&] { static_cast<void>(machine.checkpoint()); }));
    static_cast<void>(rejected([&] { machine.restore(frozen); }));
    static_cast<void>(rejected([&] { held_cpu.step(); }));
    ++committed;
  });
  machine.set_wait_observer([&](auto /*cycle*/) {
    static_cast<void>(rejected([&] { static_cast<void>(machine.checkpoint()); }));
    static_cast<void>(rejected([&] { machine.restore(frozen); }));
    static_cast<void>(rejected([&] { machine.set_wait_observer({}); }));
    static_cast<void>(rejected([&] { machine.set_wait_period_observer({}); }));
    static_cast<void>(rejected([&] { machine.set_cycle_observer({}); }));
    static_cast<void>(rejected([&] { machine.set_timer_inputs(0); }));
    static_cast<void>(rejected([&] { machine.advance_wait(1); }));
    static_cast<void>(rejected([&] { held_cpu.step(); }));
    static_cast<void>(rejected([&] { held_cpu.reset(); }));
    static_cast<void>(rejected([&] { held_cpu.set_state({}); }));
    ++observed;
  });
  require(machine.advance_wait(1) == 1 && observed == 1 && committed == 1 &&
              !machine.timing_faulted(),
          "Caught active guards preserve a healthy native wait and cannot reenter held CPU refs");
  machine.restore(frozen);
  require(machine.advance_wait(1) == 1 && observed == 1 && committed == 1,
          "Restore drops caller-owned idle observers rather than retaining captured references");
  check_counts(machine, one_idle_counts);
}

void test_observer_failure_and_trusted_recovery() {
  Fixture fixture;
  auto &machine = fixture.machine;
  fixture.enter();
  const auto frozen = machine.checkpoint();
  unsigned calls = 0;
  machine.set_wait_observer([&calls](auto /*cycle*/) {
    if (++calls == 3) { throw blofeld::Fault("Own idle observation failure"); }
  });
  const auto error = rejected([&] { machine.advance_wait(10); });
  require(error.contains("pc=0x8001") && error.contains("ccr=0xc0") && machine.timing_faulted() &&
              machine.idle_bus_cycles() == 2 && machine.run_clock().bus_cycles() == 14 &&
              machine.run_clock().elapsed_ticks() == failed_idle_ticks,
          "Idle failure retains exactly its completed prefix and complete CPU context");
  static_cast<void>(rejected([&] { machine.cpu().step(); }));
  static_cast<void>(rejected([&] { machine.advance_wait(1); }));
  static_cast<void>(rejected([&] { static_cast<void>(machine.checkpoint()); }));
  machine.cpu().set_state({.pc = entry + 1, .sp = stack, .ccr = unmasked});
  static_cast<void>(rejected([&] { machine.cpu().step(); }));
  machine.restore(frozen);
  require(machine.advance_wait(3) == 3 && calls == 3,
          "Only coordinated trusted restore repairs failed idle time and removes observers");
  check_counts(machine, recovered_idle_counts);
  check_frame(machine);
}

void test_unknown_pads_and_clock_failures() {
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
      machine.bus().write(blofeld::hc12::usb_control, 0x40);
      machine.bus().write(blofeld::hc12::clock_select, enabled);
    }
    const auto frozen = machine.checkpoint();
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
      machine.restore(frozen);
      const auto error = rejected([&] { machine.advance_wait(1); });
      require(error.contains("pc=0x8001") && machine.timing_faulted() &&
                  machine.run_clock().bus_cycles() == reset_and_wait &&
                  machine.idle_bus_cycles() == 0,
              "Unknown SCI/timer pads and PHY qualification fail without an invented period");
      static_cast<void>(rejected([&] { machine.cpu().step(); }));
    }
  }
}

void test_unsupported_crg_gates_and_invalid_boundaries() {
  for (const unsigned bit : {2U, 4U, 0x10U, 0x20U}) {
    Fixture fixture;
    auto &machine = fixture.machine;
    fixture.enter();
    machine.bus().write(blofeld::hc12::clock_select, static_cast<std::uint8_t>(bit));
    const auto error = rejected([&] { machine.advance_wait(1); });
    require(error.contains("wait clock at 0x0039") && error.contains("pc=0x8001") &&
                machine.timing_faulted() && machine.run_clock().bus_cycles() == reset_and_wait,
            "Unsupported CRG wait/PLL modes are explicit faults rather than normal-run aliases");
  }
  Fixture fixture;
  static_cast<void>(rejected([&] { fixture.machine.advance_wait(1); }));
  require(!fixture.machine.timing_faulted(), "A rejected non-WAI call has no execution effects");
  fixture.enter();
  require(fixture.machine.advance_wait(0) == 0, "An empty valid wait slice adds no time");
  fixture.machine.bus().write(blofeld::hc12::initrg, 0x10);
  fixture.machine.bus().retire_instruction();
  fixture.machine.bus().write(relocated_clock_select, 2);
  require(rejected([&] { fixture.machine.advance_wait(1); }).contains("0x1039"),
          "Wait contract faults report the live relocated CLKSEL address");
  blofeld::McuMachine untimed(image(), BLOFELD_DEFAULT_SLA);
  static_cast<void>(rejected([&] { untimed.advance_wait(1); }));
}
}

int main() {
  try {
    test_fractional_rti_wake_and_replay();
    test_sci_timer_wait_gate_matrix();
    test_inflight_sci_pause_and_resume();
    test_external_interrupt_at_idle_boundary();
    test_capture_interrupt_at_idle_boundary();
    test_active_guards_and_callback_lifetime();
    test_observer_failure_and_trusted_recovery();
    test_unknown_pads_and_clock_failures();
    test_unsupported_crg_gates_and_invalid_boundaries();
    std::cout << "Owned WAI clocks, literal domain/phase/frame expectations and recovery passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
