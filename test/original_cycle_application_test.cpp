// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_midi.hpp"

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
constexpr unsigned idle_source_hz = 30000000;
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
  const char *diagnostic;
};

void clock(blofeld::McuBus &board,
           blofeld::McuMidiInput &input,
           const blofeld::Hc12BusCycle cycle,
           const bool capture_handover) {
  // S12CPUV2 6.6 permits unaligned internal-RAM words in one bus cycle.
  // Flash/external stretching and clock-source transitions are unqualified.
  if (cycle.size == 2 && cycle.address && (*cycle.address & 1U) != 0 &&
      (*cycle.address < ram_begin || *cycle.address > last_ram_word)) {
    throw blofeld::Fault("Unqualified unaligned non-RAM word timing");
  }
  input.advance_sci(board, cycle.context, 1);
  if (capture_handover) { board.sample_timer_inputs(1); }
  board.advance_timer(1);
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

void run_case(const blofeld::Firmware &firmware,
              std::ofstream &trace,
              const ApplicationCase &item) {
  blofeld::McuBus board(firmware.mcu);
  const auto initial = board.power_off_flash_image();
  // Explicit synthetic observations, not a hardware-validated board profile.
  board.set_pin_drive(blofeld::McuPort::e, {.mask = input_mask, .levels = 3});
  board.set_pin_drive(blofeld::McuPort::q, {.mask = byte_mask});
  blofeld::McuMidiInput input(idle_source_hz, blofeld::SciReceivePin::pr2);
  bool capture_handover = false;
  blofeld::Hc12CycleBus bus(board, [&board, &input, &capture_handover](const auto cycle) {
    clock(board, input, cycle, capture_handover);
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
  const auto final = board.power_off_flash_image();
  require(initial.bytes == final.bytes && initial.known == final.known &&
              initial.fresh == final.fresh,
          "Original timed application and failure must preserve MCU NVM and erase history");
  const auto cycles = bus.cycles();
  bool latched = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &fault) {
    latched = std::string(fault.what()).contains("requires coordinated recovery");
  }
  require(latched && bus.cycles() == cycles,
          "A failed timed transaction must reject continued execution without extra clocks");
  trace << "capture-handover=" << item.capture_handover << " instructions=" << instructions
        << " cycles=" << cycles << " fault=" << error << '\n';
  board.set_observer({});
}

constexpr ApplicationCase unobserved_capture{.capture_handover = false,
                                             .instructions = 90340,
                                             .cycles = 233807,
                                             .pc = 0x87ea,
                                             .diagnostic = "Unknown active timer capture inputs"};
// Independent untimed PC/CCR census and manual mnemonic/address-mode costs:
// 4850617 retired cycles + 5 reset + 1 failing PU read = 4850623.
constexpr ApplicationCase high_capture_handover{.capture_handover = true,
                                                .instructions = 1672474,
                                                .cycles = 4850623,
                                                .pc = 0xb98d,
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
                 "totals, unchanged NVM and latched recovery; conditional fixture only.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
