// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned timebase = 1080000000, extal = 12000000, power = 108000;
constexpr unsigned control = 0xffffbd, direction = 0xffffbe, function = 0xffffbf;
constexpr unsigned f_data = 0xfffff5, f_direction = 0xfffff6, f_function = 0xfffff7;
constexpr unsigned bus_control = 0xfffffb, reset_ticks = 5400, interval = 100;
constexpr unsigned inactive = 0x300, command = 0x700, write_low = 0x500, byte_mask = 0xff;
constexpr unsigned sample_value = 0x42, native_pc = 0;

void transition_table() {
  // Rows/columns: neutral, write, read, inactive. CLE/ALE are independent.
  constexpr std::array allowed{std::array{true, false, false, true},
                               std::array{false, true, true, true},
                               std::array{false, true, true, true},
                               std::array{true, true, true, true}};
  constexpr std::array logical{0x300U, 0x100U, 0x200U, 0x300U};
  constexpr unsigned modes = 16;
  constexpr unsigned shift = 8;
  constexpr unsigned strobe_mask = 3;
  constexpr unsigned qualifiers = 0xc00;
  for (const auto contract :
       {NandBusContract::ordered_pins, NandBusContract::functional_transactions}) {
    for (unsigned old = 0; old < modes; ++old) {
      for (unsigned next = 0; next < modes; ++next) {
        NandBusDecode decoder(contract);
        static_cast<void>(decoder.observe(old << shift));
        const auto saved = decoder;
        const bool functional = contract == NandBusContract::functional_transactions;
        if (functional && !allowed[old & strobe_mask][next & strobe_mask]) {
          require(rejects([&] {
                    static_cast<void>(decoder.observe(next << shift));
                  }).contains("operation=decode at=X:0xffffbd") &&
                      decoder == saved,
                  "Incomplete transfers fail before changing the functional decoder");
        } else {
          const auto expected = functional
                                    ? ((next << shift) & qualifiers) | logical[next & strobe_mask]
                                    : next << shift;
          require(decoder.observe(next << shift) == expected &&
                      decoder.neutral() == (functional && (next & strobe_mask) == 0) &&
                      decoder.contract() == contract,
                  "Both contracts match the independent transition and logical-word tables");
        }
      }
    }
  }
  NandBusDecode decoder(NandBusContract::functional_transactions);
  const auto saved = decoder;
  constexpr unsigned outside = 0x1000;
  require(!rejects([&] { static_cast<void>(decoder.observe(outside)); }).empty() &&
              decoder == saved,
          "Unmodeled control bits cannot mutate decoder state");
}

Nand device() {
  NandArray cells;
  constexpr std::array<std::uint8_t, 1> data{sample_value};
  cells.observe_bytes({.page = 0, .column = 0}, data);
  return {cells.image(), NandReadContinuation::k9f1208_j};
}

constexpr DspRunClockInputs clocks{.extal_hz = extal,
                                   .timebase_hz = timebase,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};

void x(OwnDspMachine &machine, const unsigned address, const unsigned value) {
  machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
}

void y(OwnDspMachine &machine, const unsigned address, const unsigned value) {
  machine.bus().write({.space = DspSpace::y, .address = address, .value = value});
}

void configure(OwnDspMachine &machine) {
  for (const auto port : {DspPort::c, DspPort::f}) {
    machine.bus().gpio(port).set_contract(DspGpioContract::logical_pins);
  }
  constexpr unsigned bus = 0x1ffff;
  constexpr unsigned outputs = 0xf00;
  constexpr unsigned inputs = 0xff;
  constexpr unsigned data_function = 0x300;
  constexpr unsigned data_direction = 0x4ff;
  x(machine, bus_control, bus);
  x(machine, direction, outputs);
  x(machine, function, inputs);
  x(machine, control, inactive);
  y(machine, f_function, data_function);
  y(machine, f_direction, data_direction);
  y(machine, f_data, byte_mask);
}

void settle(NandPeer &peer, const unsigned ticks) {
  const auto events = peer.events();
  while (const auto next = events.next_ticks()) {
    if (*next > ticks) { break; }
    events.observe(*next);
  }
  peer.before_native(ticks);
}

NandPeerDetailed suffix(OwnDspMachine &machine, NandPeer &peer, const unsigned at) {
  settle(peer, at + interval);
  x(machine, control, command);
  peer.after_native(at + interval, native_pc);
  require(!peer.detailed().neutral_bus && peer.detailed().base.pending == NandPending::none,
          "Leaving neutral with both strobes high neither reads nor writes a command");
  settle(peer, at + (2 * interval));
  x(machine, control, write_low);
  peer.after_native(at + (2 * interval), native_pc);
  require(peer.detailed().base.pending == NandPending::none,
          "A real falling WE starts a pulse; it does not latch FF prematurely");
  settle(peer, at + (3 * interval));
  x(machine, control, command);
  peer.after_native(at + (3 * interval), native_pc);
  const auto reset = peer.detailed();
  const auto due = at + (3 * interval) + reset_ticks;
  require(reset.base.pending == NandPending::reset && reset.base.ready == false &&
              reset.base.next == due && !reset.trace_access && reset.column == 0,
          "Exactly the real FF write creates the protocol reset and independent Busy deadline");
  settle(peer, at + (4 * interval));
  peer.after_native(at + (4 * interval), native_pc);
  require(peer.detailed().base.next == due,
          "Stable inactive levels cannot repeat the FF command or invent a read");
  peer.events().observe(due);
  require(peer.detailed().base.ready == true && peer.detailed().base.pending == NandPending::none &&
              peer.detailed().base.cell.value == sample_value,
          "Actual reset completion releases Busy without replacing stored cells");
  return peer.detailed();
}

void peer_and_restore() {
  OwnDspMachine machine(clocks);
  NandPeer peer(machine, device(), NandCellProfile{}, NandBusContract::functional_transactions);
  configure(machine);
  peer.events().observe(power);
  peer.after_native(power + interval, native_pc);
  for (const unsigned levels : {0U, 0xc00U}) {
    settle(peer, power + (2 * interval));
    x(machine, control, levels);
    peer.after_native(power + (2 * interval), native_pc);
    require(peer.detailed().neutral_bus && peer.detailed().base.pending == NandPending::none &&
                peer.detailed().base.ready == true && !peer.detailed().trace_access &&
                peer.detailed().column == 0 && peer.detailed().base.cell.value == sample_value,
            "Neutral preserves cells, pointer and Ready without reading or latching anything");
  }
  const auto saved = peer.checkpoint();
  const auto cpu = machine.checkpoint();
  const auto before = peer.detailed();
  const auto endpoint = suffix(machine, peer, power + (2 * interval));
  OwnDspMachine fresh(clocks);
  fresh.restore(cpu);
  NandPeer copy(saved, fresh);
  copy.before_native(power + (2 * interval));
  require(copy.detailed() == before,
          "A fresh owner retains the neutral decoder and the complete NAND state");
  require(suffix(fresh, copy, power + (2 * interval)) == endpoint,
          "Fresh-owner continuation reproduces the actual reset pulse and completion");
}

void strict_control() {
  OwnDspMachine machine(clocks);
  NandPeer peer(machine, device());
  configure(machine);
  peer.events().observe(power);
  peer.after_native(power + interval, native_pc);
  settle(peer, power + (2 * interval));
  x(machine, control, 0);
  const auto fault = rejects([&] { peer.after_native(power + (2 * interval), native_pc); });
  require(peer.faulted() && fault.contains("unordered simultaneous RE/WE edges") &&
              fault.contains("pc=0x000000") && fault.contains("X:0xffffbd/Y:0xfffff5"),
          "The original ordered-pin contract retains its genuine contextual refusal");
}
}

int main() {
  try {
    transition_table();
    peer_and_restore();
    strict_control();
    std::cout
        << "512 bus transitions, neutral no-transfer, genuine reset and fresh restore passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
