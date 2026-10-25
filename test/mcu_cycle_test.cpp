// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned sci_vector = 0xffd6;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned baud_divisor = 60;
constexpr unsigned receive_interrupt = 0x24;
constexpr unsigned bus_hz = 30000000;
constexpr unsigned idle_cycles = 240;
constexpr unsigned ram_result = 0x1200;
constexpr unsigned unmasked_flags = 0xc0;
constexpr unsigned timer_count = 0x0044;
constexpr unsigned timer_enable = 0x80;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::array<std::uint8_t, 32768> flash() {
  std::array<std::uint8_t, 32768> bytes{};
  // LDS #2000; LDX #1000; DBNE X,self; CLI; NOP; BRA self.
  constexpr std::array<std::uint8_t, 14> main{
      0xcf, 0x20, 0, 0xce, 0x10, 0, 0x04, 0x35, 0xfd, 0x10, 0xef, 0xa7, 0x20, 0xfe};
  // Native status/data acknowledge, store byte and RTI.
  constexpr std::array<std::uint8_t, 10> isr{0xf6, 0, 0xcc, 0xb6, 0, 0xcf, 0x7a, 0x12, 0, 0x0b};
  std::ranges::copy(main, bytes.begin());
  std::ranges::copy(isr, bytes.begin() + handler - entry);
  bytes[sci_vector - entry] = handler >> 8U;
  bytes[reset_vector - entry] = entry >> 8U;
  return bytes;
}

void configure(blofeld::McuBus &board) {
  board.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  board.write(blofeld::hc12::sci_baud_high, 0);
  board.write(blofeld::hc12::sci_baud_low, baud_divisor);
  board.write(blofeld::hc12::sci_control2, receive_interrupt);
  board.write(blofeld::hc12::tscr1, timer_enable);
}

void test_midi_and_timer_during_native_cpu_execution() {
  blofeld::McuBus board(flash());
  configure(board);
  blofeld::McuMidiInput input(bus_hz, blofeld::SciReceivePin::pr2);
  blofeld::Hc12CycleBus bus(board, [&board, &input](const auto cycle) {
    // Explicit un-stretched internal-memory, fixed 30 MHz test contract.
    input.advance_sci(board, cycle.context, 1);
    board.advance_timer(1);
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bus.advance_idle(idle_cycles);
  constexpr std::array<std::uint8_t, 1> byte{0xa5};
  input.enqueue(byte);
  unsigned entries = 0;
  constexpr unsigned instruction_limit = 5000;
  constexpr unsigned return_pc = 0x800c;
  bool returned = false;
  for (unsigned instruction = 0; instruction < instruction_limit && !returned; ++instruction) {
    const auto result = cpu.step();
    if (result.interrupt_vector) {
      require(result.interrupt_vector == sci_vector, "Only the real SCI receive source may enter");
      ++entries;
    }
    returned = entries != 0 && cpu.state().pc == return_pc;
  }
  // 5 reset + 240 mark + LDS/LDX 4 + 4096 DBNE*3 + CLI/NOP 2
  // + IRQ 9 + three ISR operands*3 + RTI 8.
  constexpr unsigned expected_cycles = 12565;
  require(returned && entries == 1 && bus.cycles() == expected_cycles &&
              board.read(ram_result) == byte.front() && input.queued_bytes() == 0 &&
              board.read_memory({.address = timer_count, .size = 2}) == expected_cycles &&
              !cpu.pending_interrupt(),
          "Native CPU cycles must clock actual MIDI reception, timer count, IRQ, acknowledge "
          "and RTI without host receive flags or forced CPU entry");
}

void test_unqualified_live_receive_phase_fails() {
  blofeld::McuBus board(flash());
  configure(board);
  blofeld::McuMidiInput input(bus_hz, blofeld::SciReceivePin::pr2);
  blofeld::Hc12CycleBus bus(
      board, [&board, &input](const auto cycle) { input.advance_sci(board, cycle.context, 1); });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bus.advance_idle(idle_cycles);
  constexpr unsigned foreground = 0x800c;
  cpu.set_state({.pc = foreground, .sp = 0x2000, .ccr = unmasked_flags});
  constexpr std::array<std::uint8_t, 1> byte{0xa5};
  input.enqueue(byte);
  std::string error;
  constexpr unsigned limit = 4000;
  for (unsigned instruction = 0; instruction < limit && error.empty(); ++instruction) {
    try {
      cpu.step();
    } catch (const blofeld::Fault &fault) { error = fault.what(); }
  }
  require(bus.faulted() && error.contains("Unverified SCI stop-to-data flag transfer phase") &&
              error.contains("pc=0x800c") && error.contains("Interrupt selection"),
          "Instruction-boundary IRQ polling must expose the unresolved SCI transfer edge");
}
}

int main() {
  try {
    test_midi_and_timer_during_native_cpu_execution();
    test_unqualified_live_receive_phase_fails();
    std::cout << "CPU-clocked native MIDI/timer/IRQ/RTI and live SCI uncertainty passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
