// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "lcd_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
constexpr unsigned application = 0x8200;
constexpr unsigned release_inputs = 0x820e;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned capture_enable = 0x87e8;
constexpr unsigned ready_poll = 0xb98d;
constexpr unsigned display_initialization = 0x8b94;
constexpr unsigned first_loopback = 0x8029;
constexpr unsigned second_loopback = 0x8036;
constexpr unsigned initial_e_mask = 0xfb;
constexpr unsigned application_e_mask = 0xeb;
constexpr unsigned enable_pad = 0x10;
constexpr unsigned other_b_inputs = 0xe8;
constexpr unsigned released_b_inputs = 0x1e;
constexpr unsigned u_mask = 0x3f;
constexpr unsigned receive_pad = 4;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned last_ram_word = 0x25fe;
constexpr unsigned rti_vector = 0xfff0;
constexpr unsigned rti_handler = 0x88a5;
constexpr unsigned rti_opcode = 0x0b;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned instruction_limit = 2500000;
constexpr unsigned expected_instructions = 1944294;
constexpr unsigned expected_entry_cycles = 5615369;
constexpr unsigned expected_cycles = 5618487;
constexpr unsigned expected_ticks = 11237638;
constexpr unsigned expected_edges = 2247527;
constexpr unsigned rti_remaining = 638;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Observations {
  bool phy = false;
  bool capture = false;
};

void observe_inputs(blofeld::McuBus &board, const unsigned pc, Observations &observed) {
  if (pc == phy_poll && !observed.phy) {
    board.observe_phy({.edges = 15});
    observed.phy = true;
  }
  if (pc == first_loopback || pc == second_loopback) {
    const auto sample = board.observe_pads(blofeld::McuPort::e, 4);
    require(sample.known == 4, "E2 must be driven before its explicit E3 loopback observation");
    const auto levels = static_cast<std::uint8_t>(3U | ((sample.levels & 4U) != 0 ? 8U : 0U));
    board.set_pin_drive(blofeld::McuPort::e, {.mask = initial_e_mask, .levels = levels});
  }
  if (pc == application) {
    board.set_pin_drive(blofeld::McuPort::q, {.floating = byte_mask});
    board.observe_usb_decoder({});
    board.set_pin_drive(blofeld::McuPort::e,
                        {.mask = application_e_mask, .levels = 3, .floating = enable_pad});
    board.set_pin_drive(blofeld::McuPort::b, {.mask = other_b_inputs});
  }
  if (pc == release_inputs) {
    board.set_pin_drive(blofeld::McuPort::b, {.mask = released_b_inputs});
  }
  if (pc == capture_enable && !observed.capture) {
    board.set_pin_drive(blofeld::McuPort::t, {.mask = 1, .levels = 1});
    board.sample_timer_inputs(1);
    observed.capture = true;
  }
  if (pc == ready_poll) { board.set_pin_drive(blofeld::McuPort::u, {.mask = u_mask, .levels = 1}); }
  if (pc == display_initialization) { board.set_pin_drive(blofeld::McuPort::b, {.mask = 8}); }
}

void advance(blofeld::McuBus &board,
             blofeld::McuRunClock &clock,
             const Observations observed,
             const blofeld::Hc12BusCycle cycle) {
  if (cycle.size == 2 && cycle.address && (*cycle.address & 1U) != 0 &&
      (*cycle.address < ram_begin || *cycle.address > last_ram_word)) {
    throw blofeld::Fault("Unqualified unaligned non-RAM word timing");
  }
  const auto idle = board.observe_pads(blofeld::McuPort::r, receive_pad);
  require(idle.known == receive_pad && idle.levels == receive_pad,
          "The external PR2 idle observation must remain known without substituting SCI flags");
  if (observed.capture) { board.sample_timer_inputs(1); }
  clock.advance_run_cycle(board, cycle.context);
}

bool same_state(const blofeld::CpuState &left, const blofeld::CpuState &right) {
  return left.pc == right.pc && left.sp == right.sp && left.d == right.d && left.x == right.x &&
         left.y == right.y && left.ccr == right.ccr;
}

void check_irq_frame(blofeld::McuBus &board,
                     const blofeld::CpuState &foreground,
                     const blofeld::CpuState &entered) {
  constexpr unsigned frame_bytes = 9;
  require(foreground.sp >= ram_begin + frame_bytes && entered.sp == foreground.sp - frame_bytes &&
              entered.pc == rti_handler,
          "Original RTI entry must use its actual vector and native nine-byte frame");
  // Independent S12CPUV2 saved layout: CCR, B, A, X high/low, Y high/low, PC high/low.
  const std::array<unsigned, frame_bytes> bytes{foreground.ccr,
                                                foreground.d & byte_mask,
                                                static_cast<unsigned>(foreground.d) >> 8U,
                                                static_cast<unsigned>(foreground.x) >> 8U,
                                                foreground.x & byte_mask,
                                                static_cast<unsigned>(foreground.y) >> 8U,
                                                foreground.y & byte_mask,
                                                static_cast<unsigned>(foreground.pc) >> 8U,
                                                foreground.pc & byte_mask};
  for (unsigned index = 0; index < bytes.size(); ++index) {
    require(board.read(static_cast<std::uint16_t>(entered.sp + index)) == bytes.at(index),
            "Every original interrupt-frame byte must match the independently specified order");
  }
}

void check_return(blofeld::McuBus &board,
                  const blofeld::Hc12 &cpu,
                  const blofeld::Hc12CycleBus &timed,
                  const blofeld::McuRunClock &clock,
                  const unsigned instructions) {
  constexpr unsigned crg_flags = 0x37;
  constexpr unsigned realtime_flag = 0x80;
  require(instructions == expected_instructions && timed.cycles() == expected_cycles &&
              clock.bus_cycles() == expected_cycles && clock.elapsed_ticks() == expected_ticks &&
              clock.oscillator_cycles() == expected_edges &&
              board.oscillator_cycles_until_rti() == rti_remaining &&
              (board.read(crg_flags) & realtime_flag) == 0 && !cpu.pending_interrupt() &&
              !cpu.waiting() && !clock.faulted() && !timed.faulted(),
          "Original first RTI must acknowledge its real oscillator flag and preserve exact time");
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  blofeld::McuBus board(firmware.mcu);
  const auto initial = board.power_off_flash_image();
  board.set_pin_drive(blofeld::McuPort::e, {.mask = initial_e_mask, .levels = 3});
  board.set_pin_drive(blofeld::McuPort::q, {.mask = byte_mask});
  board.set_pin_drive(blofeld::McuPort::r, {.mask = receive_pad, .levels = receive_pad});
  blofeld::test::LcdFixture display(board);
  blofeld::McuRunClock clock(
      {.oscillator_hz = oscillator_hz,
       .initial_oscillator_phase_ticks = 0,
       .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary});
  Observations observed;
  blofeld::Hc12CycleBus timed(board, [&board, &clock, &observed](const auto cycle) {
    advance(board, clock, observed, cycle);
  });
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  board.set_observer([&display, &trace, &timed](const auto &event) {
    display.observe(event);
    trace << "cycle=" << timed.cycles() << " write=" << blofeld::hex(event.address) << ':'
          << blofeld::hex(event.value, 2) << '\n';
  });
  blofeld::Hc12 cpu(timed, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  unsigned instructions = 0;
  unsigned entries = 0;
  bool returned = false;
  blofeld::CpuState foreground;
  for (unsigned steps = 0; steps < instruction_limit && !returned; ++steps) {
    const auto before = cpu.state();
    observe_inputs(board, before.pc, observed);
    const auto opcode = board.peek(before.pc);
    const auto result = cpu.step();
    if (result.instruction_executed) { ++instructions; }
    if (result.interrupt_vector) {
      require(entries == 0 && result.interrupt_vector == rti_vector &&
                  !result.instruction_executed && !result.stack_reused &&
                  timed.cycles() == expected_entry_cycles,
              "Actual oscillator time must request original RTI without injected CPU entry");
      ++entries;
      foreground = before;
      check_irq_frame(board, before, cpu.state());
    }
    if (entries == 1 && opcode == rti_opcode && result.instruction_executed) {
      require(!result.interrupt_vector && same_state(foreground, cpu.state()),
              "Original RTI must restore the complete foreground state without host replacement");
      returned = true;
    }
  }
  require(returned && observed.phy && observed.capture && entries == 1,
          "The original startup must complete its first real oscillator-driven handler");
  check_return(board, cpu, timed, clock, instructions);
  const auto final = board.power_off_flash_image();
  require(initial.bytes == final.bytes && initial.known == final.known &&
              initial.fresh == final.fresh,
          "Unmodified reset, peripheral setup and native RTI must preserve all MCU NVM/history");
  trace << "retired-instructions=" << instructions << " bus-cycles=" << timed.cycles()
        << " timebase-ticks=" << clock.elapsed_ticks()
        << " oscillator-cycles=" << clock.oscillator_cycles()
        << " pc=" << blofeld::hex(cpu.state().pc) << '\n';
  board.set_observer({});
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-run-clock firmware.zip private.trace");
    }
    run({.firmware = arguments[1], .trace = arguments[2]});
    std::cout << "Original MCU: first RTI entry/acknowledge/return from actual CPU/oscillator "
                 "time; no foreground-slot period injection. Conditional reference clock, "
                 "GPIO/PHY/LCD/ready experiment; full board boot remains unproven.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
