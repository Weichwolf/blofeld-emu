// SPDX-License-Identifier: GPL-3.0-only
#include "io/dsp_reset.hpp"
#include "io/midi_port.hpp"
#include "io/shi.hpp"
#include "mcu/devices.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/ports.hpp"
#include "mcu/state_probe.hpp"
#include "storage/flash_image.hpp"

#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace {
using namespace blofeld::vi;
using mcu::StateProbe;

void require(const bool passed, const char *message) {
  if (!passed) { throw std::runtime_error(message); }
}

struct Fixture {
  FlashImage flash;
  std::stop_source stop;
  const std::stop_token token = stop.get_token();
  mcu::Memory memory{flash};
  mcu::Devices devices{token};
  Shi shi;
  DspReset reset;
  mcu::Io io{memory, flash, devices, shi, reset, token, nullptr, nullptr, nullptr};
  std::vector<cpu12::Ram> grants = memory.regions();
  cpu12::Core core{cpu12::Memory::mapped(grants)};

  [[nodiscard]] StateProbe::Record capture() const {
    return StateProbe::capture(core, memory, devices, io, 0);
  }
};

void semantic_storage() {
  Fixture first;
  Fixture second;
  const auto before = first.capture();
  require(before.parts == second.capture().parts,
          "Canonical MCU state ignores object addresses and padding");
  const auto saved = first.memory.save();
  constexpr cpu12::Address address = 0x1800;
  require(first.memory.write({.address = address, .value = 1}), "Write real MCU SRAM");
  require(first.capture().parts[1] != before.parts[1], "SRAM data and knowledge affect its digest");
  first.memory.restore(saved);
  auto changed = saved;
  changed.system_known[0] ^= 1;
  first.memory.restore(changed);
  require(first.capture().parts[1] != before.parts[1], "Knowledge alone affects its digest");
  first.memory.restore(saved);
  require(first.capture().parts == before.parts, "A restored snapshot has the same full digest");
  require(first.devices.timer.advance(1), "Advance real timer state");
  require(first.capture().parts[2] != before.parts[2], "Hidden timer time affects its digest");
  require(second.devices.clock.advance(1), "Advance real clock state");
  require(second.capture().parts[3] != before.parts[3], "Clock time affects its digest");
  require(second.devices.sci.advance(1), "Advance real SCI state");
  require(second.capture().parts[4] != before.parts[4], "SCI continuation affects its digest");
  constexpr std::uint16_t interrupts = 0x304;
  constexpr std::uint16_t control = 0x4000;
  require(first.devices.usb.write({.offset = interrupts, .value = 1, .bytes = 2}, control),
          "Write a real USB interrupt register");
  require(first.capture().parts[5] != before.parts[5], "USB register state affects its digest");
  auto local = first.io.save();
  local.ports.state.ports[0].latch = 1;
  first.io.restore(local);
  require(first.capture().parts[6] != before.parts[6], "GPIO latch state affects its digest");
  constexpr std::uint16_t divider = 0x100;
  require(local.flash.write({.offset = divider, .value = 1}), "Write actual flash control state");
  first.io.restore(local);
  require(first.capture().parts[7] != before.parts[7], "Flash control state affects its digest");
  auto display = second.io.save();
  mcu::Ports ports(second.shi);
  const auto put = [&](const mcu::Ports::Write access) {
    require(ports.write(access) && display.display.write(ports, access), "Drive actual LCD pins");
  };
  constexpr std::uint8_t outputs = 0xe1;
  constexpr std::uint8_t command = 0x40;
  constexpr std::uint8_t all_bits = 0xff;
  constexpr std::uint8_t on = 0xaf;
  constexpr std::uint8_t enable = 0x10;
  put({.offset = 3, .value = outputs});
  put({.offset = 1, .value = command});
  put({.offset = 2, .value = all_bits});
  put({.offset = 0, .value = on});
  put({.offset = 8, .value = enable});
  put({.offset = 8});
  second.io.restore(display);
  require(second.capture().parts[8] != before.parts[8], "LCD commands affect their digest");
  require(first.reset.assert_reset(), "Assert the actual DSP reset source");
  require(first.capture().parts[0] != before.parts[0], "Reset epoch and level affect their digest");
  second.core.set_registers({.d = 1});
  require(second.capture().parts[0] != before.parts[0],
          "Architectural registers affect their digest");
}

void transactions() {
  StateProbe probe;
  require(StateProbe::active() == nullptr, "No implicit diagnostic owner");
  {
    const StateProbe::Scope scope(&probe);
    require(StateProbe::active() == &probe, "The diagnostic belongs to its native owner");
    StateProbe::Record first{.instructions = 1, .cycles = 2, .ticks = 3};
    probe.executed(2, false);
    probe.wait(first, true);
    require(probe.count() == 0 && probe.pending() == 1, "Speculative state is not published");
    probe.rollback(1, true);
    require(probe.count() == 0 && probe.pending() == 0, "Rollback discards unconfirmed state");
    probe.executed(1, true);
    probe.wait(first, true);
    probe.commit(first.instructions, {.estimated_cycles = first.cycles}, first.ticks);
    require(probe.count() == 1 && probe.pending() == 0, "A confirmed wait is published once");
    first.instructions = 2;
    probe.wait(first, false);
    require(probe.count() == 2, "Non-speculative state needs no future confirmation");
    probe.finish(first);
  }
  require(StateProbe::active() == nullptr, "Leaving the owner clears diagnostic context");
  std::ostringstream output;
  probe.report(output);
  require(output.str().contains("executed=3 discarded=1") &&
              output.str().contains("rollbacks=1 reinstalls=0") &&
              output.str().contains("replay-executed=1") &&
              output.str().contains("waits=2 pending-waits=0") &&
              output.str().contains("MCU_WAIT ordinal=2 instructions=2") &&
              output.str().contains("MCU_EXIT"),
          "Committed, discarded, replayed and exit state remain distinct");
}

void common_stop() {
  std::stop_source stop;
  StateProbe probe(&stop);
  constexpr std::uint64_t cut = MidiPort::reference_hz;
  probe.end_at(0);
  require(!probe.stopping(cut - 1) && probe.stopping(cut),
          "Qualification uses a shared guest cut after one explicit drain second");
  const StateProbe::Record boundary{.instructions = 1, .cycles = 2, .ticks = cut};
  probe.wait(boundary, true);
  require(!stop.stop_requested(), "An unconfirmed ending cannot stop either owner");
  probe.rollback(1, true);
  probe.commit(0, {}, 0);
  require(!stop.stop_requested(), "Rollback revokes an ending candidate");
  probe.wait(boundary, true);
  bool rejected = false;
  try {
    probe.commit(boundary.instructions, {.estimated_cycles = boundary.cycles}, cut);
  } catch (const std::logic_error &) { rejected = true; }
  require(rejected && !stop.stop_requested(), "An ending cannot drop unfinished original PCM");
  probe.pcm_complete();
  probe.commit(boundary.instructions, {.estimated_cycles = boundary.cycles}, cut);
  require(stop.stop_requested(),
          "Only a confirmed common boundary after complete PCM stops owners");
}
}

int main() try {
  semantic_storage();
  transactions();
  common_stop();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
