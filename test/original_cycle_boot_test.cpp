// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
constexpr unsigned application = 0x8200;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned input_mask = 0xfb;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_end = 0x2600;
constexpr unsigned expected_instructions = 90314;
constexpr unsigned expected_cycles = 233749;
constexpr unsigned idle_source_hz = 30000000;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Inputs {
  std::filesystem::path reference;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.reference);
  blofeld::McuBus board(firmware.mcu);
  const auto initial = board.power_off_flash_image();
  board.set_pin_drive(blofeld::McuPort::e, {.mask = input_mask});
  blofeld::McuMidiInput input(idle_source_hz, blofeld::SciReceivePin::ps4);
  blofeld::Hc12CycleBus bus(board, [&board, &input](const auto cycle) {
    // Only core/bus cycles are supplied. PHY qualification remains explicit;
    // no oscillator ratio, analog startup or measured board frequency is inferred.
    if (cycle.size == 2 && cycle.address && (*cycle.address & 1U) != 0) {
      throw blofeld::Fault("Unqualified unaligned word timing in cold-boot contract");
    }
    input.advance_sci(board, cycle.context, 1);
    board.advance_timer(1);
  });
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  board.set_observer([&trace, &bus](const auto &event) {
    trace << "cycle=" << bus.cycles() << " write=" << blofeld::hex(event.address) << ':'
          << blofeld::hex(event.value, 2) << '\n';
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bool phy_observed = false;
  bool ram_cleared = false;
  unsigned count = 0;
  while (cpu.state().pc != application && count < expected_instructions) {
    constexpr unsigned clear_end = 0x807d;
    if (cpu.state().pc == clear_end) {
      for (unsigned address = ram_begin; address < ram_end; ++address) {
        require(board.read(static_cast<std::uint16_t>(address)) == 0,
                "Original native cold loop must initialize every RAM byte while timers/SCI run");
      }
      ram_cleared = true;
    }
    if (cpu.state().pc == phy_poll && !phy_observed) {
      board.observe_phy({.edges = 15});
      phy_observed = true;
    }
    cpu.step();
    ++count;
  }
  require(phy_observed && ram_cleared && count == expected_instructions &&
              cpu.state().pc == application && cpu.state().sp == ram_end && cpu.state().d == 0 &&
              bus.cycles() == expected_cycles,
          "Original reset-to-application must match independent instruction and core-cycle totals");
  const auto final = board.power_off_flash_image();
  require(initial.bytes == final.bytes && initial.known == final.known &&
              initial.fresh == final.fresh,
          "Clocked original boot must preserve MCU NVM and erase history");
  trace << "instructions=" << count << " cycles=" << bus.cycles()
        << " pc=" << blofeld::hex(cpu.state().pc) << '\n';
  board.set_observer({});
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-cycle-boot firmware.zip private.trace");
    }
    run({.reference = arguments[1], .trace = arguments[2]});
    std::cout
        << "Original MCU bootloader: 90314 native instructions, 233749 un-stretched core "
           "cycles including reset, running SCI/timer, RAM clear/checksum and NVM preservation. "
           "Conditional pad/PHY/internal-memory contract; full scheduled board remains open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
