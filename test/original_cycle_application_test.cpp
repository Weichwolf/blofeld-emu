// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned capture_enable = 0x87e8;
constexpr unsigned input_mask = 0xfb;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned last_ram_word = 0x25fe;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned reference_timebase_hz = 60000000;
constexpr unsigned receive_pad = 4;
constexpr unsigned instruction_limit = 2000000;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Inputs {
  std::filesystem::path reference;
  std::filesystem::path trace;
};

struct ApplicationCase {
  bool capture_handover;
  unsigned instructions;
  unsigned cycles;
  unsigned pc;
  unsigned timebase_ticks;
  unsigned oscillator_cycles;
  unsigned rti_remaining;
  const char *diagnostic;
};

void clock(blofeld::McuBus &board,
           blofeld::McuRunClock &run_clock,
           const blofeld::Hc12BusCycle cycle,
           const bool capture_handover) {
  // S12CPUV2 6.6 permits unaligned internal-RAM words in one bus cycle.
  // Flash/external stretching and analog clock switching remain unqualified.
  if (cycle.size == 2 && cycle.address && (*cycle.address & 1U) != 0 &&
      (*cycle.address < ram_begin || *cycle.address > last_ram_word)) {
    throw blofeld::Fault("Unqualified unaligned non-RAM word timing");
  }
  const auto idle = board.observe_pads(blofeld::McuPort::r, receive_pad);
  require(idle.known == receive_pad && idle.levels == receive_pad,
          "PR2 remains an explicitly observed idle input across native source-clock changes");
  if (capture_handover) { board.sample_timer_inputs(1); }
  run_clock.advance_run_cycle(board, cycle.context);
}

void loopback(blofeld::McuBus &board, const unsigned pc) {
  constexpr unsigned first_sample = 0x8029;
  constexpr unsigned second_sample = 0x8036;
  if (pc != first_sample && pc != second_sample) { return; }
  const auto sample = board.observe_pads(blofeld::McuPort::e, 4);
  require(sample.known == 4, "E2 must be driven before the explicit E2/E3 loopback is sampled");
  const auto levels = static_cast<std::uint8_t>(3U | ((sample.levels & 4U) != 0 ? 8U : 0U));
  board.set_pin_drive(blofeld::McuPort::e, {.mask = input_mask, .levels = levels});
}

void check_failure(const ApplicationCase &item,
                   const unsigned instructions,
                   const blofeld::Hc12CycleBus &bus,
                   const blofeld::CpuState state,
                   const std::string &error) {
  require(instructions == item.instructions && bus.cycles() == item.cycles && state.pc == item.pc &&
              bus.faulted() && error.contains(item.diagnostic) &&
              error.contains("pc=" + blofeld::hex(state.pc)) &&
              error.contains("ccr=" + blofeld::hex(state.ccr, 2)),
          "Original application must expose the expected timed hardware frontier with context");
  if (item.capture_handover) {
    constexpr unsigned stack = 0x25ec;
    require(state.sp == stack && error.contains("MCU read at 0x0278") &&
                error.contains("mask=0x3f"),
            "PU input must stay unknown, with actual native read address and mask");
  } else {
    require(error.contains("MCU timer advance at 0x0040") && error.contains("mask=0x01"),
            "Unobserved capture input must expose channel, operation and timer address");
  }
}

void check_clock(const ApplicationCase &item,
                 const blofeld::McuRunClock &clock,
                 blofeld::McuBus &board) {
  require(clock.timebase_hz() == reference_timebase_hz && clock.bus_cycles() == item.cycles &&
              clock.elapsed_ticks() == item.timebase_ticks &&
              clock.oscillator_cycles() == item.oscillator_cycles &&
              board.oscillator_cycles_until_rti() == item.rti_remaining &&
              clock.faulted() == !item.capture_handover,
          "Native clock-source writes must retain independent elapsed time and oscillator phase");
  constexpr unsigned crg_flags = 0x37;
  constexpr unsigned rti_flag = 0x80;
  require((board.read(crg_flags) & rti_flag) == (item.capture_handover ? rti_flag : 0U),
          "RTIF comes from elapsed CPU/oscillator time, not foreground poll observations");
}

void run_case(const blofeld::Firmware &firmware,
              std::ofstream &trace,
              const ApplicationCase &item) {
  blofeld::McuBus board(firmware.mcu);
  const auto initial = board.power_off_flash_image();
  // Explicit synthetic observations, not a hardware-validated board profile.
  board.set_pin_drive(blofeld::McuPort::e, {.mask = input_mask, .levels = 3});
  board.set_pin_drive(blofeld::McuPort::q, {.mask = byte_mask});
  board.set_pin_drive(blofeld::McuPort::r, {.mask = receive_pad, .levels = receive_pad});
  blofeld::McuRunClock run_clock(
      {.oscillator_hz = oscillator_hz,
       .initial_oscillator_phase_ticks = 0,
       .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary});
  bool capture_handover = false;
  blofeld::Hc12CycleBus bus(board, [&board, &run_clock, &capture_handover](const auto cycle) {
    clock(board, run_clock, cycle, capture_handover);
  });
  board.set_observer([&trace, &bus](const auto &event) {
    trace << "cycle=" << bus.cycles() << " write=" << blofeld::hex(event.address) << ':'
          << blofeld::hex(event.value, 2) << '\n';
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bool phy = false;
  unsigned instructions = 0;
  std::string error;
  while (instructions < instruction_limit && error.empty()) {
    const auto state = cpu.state();
    loopback(board, state.pc);
    if (state.pc == phy_poll && !phy) {
      board.observe_phy({.edges = 15});
      phy = true;
    }
    if (state.pc == capture_enable && item.capture_handover && !capture_handover) {
      // Attach a constant high input only after native PT0 output is high,
      // before its native capture-enable write. Never inject an edge/flag.
      board.set_pin_drive(blofeld::McuPort::t, {.mask = 1, .levels = 1});
      board.sample_timer_inputs(1);
      capture_handover = true;
    }
    try {
      cpu.step();
      ++instructions;
    } catch (const blofeld::Fault &fault) { error = fault.what(); }
  }
  require(phy && capture_handover == item.capture_handover,
          "Original code must reach the explicit PHY/capture observation contracts");
  check_failure(item, instructions, bus, cpu.state(), error);
  check_clock(item, run_clock, board);
  const auto final = board.power_off_flash_image();
  require(initial.bytes == final.bytes && initial.known == final.known &&
              initial.fresh == final.fresh,
          "Original timed application and failure must preserve MCU NVM and erase history");
  const auto cycles = bus.cycles();
  const auto elapsed = run_clock.elapsed_ticks();
  bool latched = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &fault) {
    latched = std::string(fault.what()).contains("requires coordinated recovery");
  }
  require(latched && bus.cycles() == cycles && run_clock.elapsed_ticks() == elapsed,
          "A failed timed transaction must reject continued execution without extra clocks");
  trace << "capture-handover=" << item.capture_handover << " instructions=" << instructions
        << " cycles=" << cycles << " timebase-ticks=" << elapsed
        << " oscillator-cycles=" << run_clock.oscillator_cycles()
        << " rti-remaining=" << board.oscillator_cycles_until_rti() << " fault=" << error << '\n';
  board.set_observer({});
}

constexpr ApplicationCase unobserved_capture{.capture_handover = false,
                                             .instructions = 90340,
                                             .cycles = 233807,
                                             .pc = 0x87ea,
                                             .timebase_ticks = 468214,
                                             .oscillator_cycles = 93642,
                                             .rti_remaining = 0,
                                             .diagnostic = "Unknown active timer capture inputs"};
// Independent untimed PC/CCR census and manual mnemonic/address-mode costs:
// 4850617 retired cycles + 5 reset + 1 failing PU read = 4850623.
// CLKSEL writes at cycles 75, 233898, 233906: 83 OSC cycles at 10 ticks,
// remaining PHY cycles at 2 ticks; 5 ticks per 12 MHz oscillator edge.
// RTICTL=40 starts after cycle 233874: 1846713 later OSC edges leave 4679 to timeout.
constexpr ApplicationCase high_capture_handover{.capture_handover = true,
                                                .instructions = 1672474,
                                                .cycles = 4850623,
                                                .pc = 0xb98d,
                                                .timebase_ticks = 9701910,
                                                .oscillator_cycles = 1940382,
                                                .rti_remaining = 4679,
                                                .diagnostic = "Unknown MCU GPIO input"};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.reference);
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  run_case(firmware, trace, unobserved_capture);
  run_case(firmware, trace, high_capture_handover);
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-cycle-application firmware.zip private.trace");
    }
    run({.reference = arguments[1], .trace = arguments[2]});
    std::cout << "Original timed application: strict capture/PU failures, independent cycle "
                 "totals, actual oscillator/RTI phase, unchanged NVM and latched recovery; "
                 "conditional fixture only.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
