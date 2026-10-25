// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_midi.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <string_view>

namespace {
constexpr unsigned application = 0x8200;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned first_loopback = 0x8029, second_loopback = 0x8036;
constexpr unsigned input_mask = 0xfb, probe_output = 4, probe_input = 8;
constexpr unsigned port_mask = 0xff;
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

struct Profile {
  blofeld::SciReceivePin receive;
  std::string_view suffix;
  bool loopback = false;
  bool missing_receive = false;
};

constexpr std::array profiles{
    Profile{.receive = blofeld::SciReceivePin::ps4, .suffix = ""},
    Profile{.receive = blofeld::SciReceivePin::pr2, .suffix = ".100pin", .loopback = true},
    Profile{.receive = blofeld::SciReceivePin::ps4,
            .suffix = ".missing-source",
            .loopback = true,
            .missing_receive = true}};

void check_nvm(const blofeld::McuBus &board, const blofeld::McuFlashImage &initial) {
  const auto final = board.power_off_flash_image();
  require(initial.bytes == final.bytes && initial.known == final.known &&
              initial.fresh == final.fresh,
          "Clocked original boot must preserve MCU NVM and erase history");
}

void observe_probe(blofeld::McuBus &board, const unsigned pc, const Profile profile) {
  if (!profile.loopback || (pc != first_loopback && pc != second_loopback)) { return; }
  const auto driven = board.observe_pads(blofeld::McuPort::e, probe_output);
  require(driven.known == probe_output, "Native PE2 must drive the declared PE3 loopback");
  board.set_pin_drive(blofeld::McuPort::e,
                      {.mask = input_mask,
                       .levels = static_cast<std::uint8_t>(
                           (driven.levels & probe_output) != 0 ? probe_input : 0U)});
}

void check_missing_source(blofeld::Hc12 &cpu,
                          const blofeld::Hc12CycleBus &bus,
                          const blofeld::Fault &fault) {
  const std::string_view message(fault.what());
  require(message.contains("pc=0x8078") &&
              message.contains("Unknown MCU GPIO input at 0x0268 mask=0x04"),
          "The original large-package branch must sample missing PR2 instead of aliasing PS4");
  const auto cycles = bus.cycles();
  const auto pc = cpu.state().pc;
  bool refused = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &) { refused = true; }
  require(refused && bus.cycles() == cycles && cpu.state().pc == pc,
          "The failed native receive sample cannot resume or consume another cycle");
}

void run(const Inputs &inputs, const Profile profile) {
  const auto firmware = blofeld::Firmware::load(inputs.reference);
  blofeld::McuBus board(firmware.mcu);
  const auto initial = board.power_off_flash_image();
  board.set_pin_drive(blofeld::McuPort::e, {.mask = input_mask});
  // Isolated large-package context: native PERQ/PPSQ bias the undriven Q inputs.
  if (profile.loopback) { board.set_pin_drive(blofeld::McuPort::q, {.floating = port_mask}); }
  blofeld::McuMidiInput input(idle_source_hz, profile.receive);
  blofeld::Hc12CycleBus bus(board, [&board, &input](const auto cycle) {
    // Logical CPU bus periods; PHY startup and physical wiring remain explicit inputs.
    if (cycle.size == 2 && cycle.address && (*cycle.address & 1U) != 0) {
      throw blofeld::Fault("Unqualified unaligned word timing in cold-boot contract");
    }
    input.advance_sci(board, cycle.context, 1);
    board.advance_timer(1);
  });
  auto path = inputs.trace;
  path += profile.suffix;
  std::ofstream trace(path);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  unsigned routing_writes = 0;
  board.set_observer([&trace, &bus, &routing_writes](const auto &event) {
    routing_writes += static_cast<unsigned>(event.address == blofeld::hc12::modrr);
    trace << "cycle=" << bus.cycles() << " write=" << blofeld::hex(event.address) << ':'
          << blofeld::hex(event.value, 2) << '\n';
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bool phy_observed = false;
  bool ram_cleared = false;
  unsigned count = 0;
  // S12CPUV2: skip LDAB IMM(1) + STAB EXT(3), but take BNE(3) instead of P(1).
  const unsigned instructions = expected_instructions - (profile.loopback ? 2U : 0U);
  const unsigned cycles = expected_cycles - (profile.loopback ? 1U + 3U - 2U : 0U);
  while (cpu.state().pc != application && count < instructions) {
    constexpr unsigned clear_end = 0x807d;
    observe_probe(board, cpu.state().pc, profile);
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
    try {
      cpu.step();
    } catch (const blofeld::Fault &fault) {
      if (!profile.missing_receive) { throw; }
      require(phy_observed && routing_writes == 0 && board.read(blofeld::hc12::modrr) == 0,
              "Real bootstrap routing stays at reset-zero before the missing input fault");
      check_missing_source(cpu, bus, fault);
      check_nvm(board, initial);
      trace << "missing-pr2 instructions=" << count << " cycles=" << bus.cycles()
            << " pc=" << blofeld::hex(cpu.state().pc) << '\n';
      board.set_observer({});
      return;
    }
    ++count;
  }
  require(!profile.missing_receive && phy_observed && ram_cleared && count == instructions &&
              cpu.state().pc == application && cpu.state().sp == ram_end && cpu.state().d == 0 &&
              bus.cycles() == cycles && routing_writes == (profile.loopback ? 0U : 1U) &&
              board.read(blofeld::hc12::modrr) == (profile.loopback ? 0U : 3U),
          "Both original bootstrap branches retain independent instruction/cycle/routing results");
  check_nvm(board, initial);
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
    for (const auto profile : profiles) {
      run({.reference = arguments[1], .trace = arguments[2]}, profile);
    }
    std::cout << "Original MCU bootloader: 90314/90312 instructions, 233749/233747 un-stretched "
                 "cycles including reset, native PS4/PR2 routing, RAM clear/checksum and NVM "
                 "preservation. Missing PR2 rejects. Conditional pad/PHY/internal-memory contract; "
                 "full scheduled board remains open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
