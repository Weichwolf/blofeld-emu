// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_instruction_boundary};
constexpr Word boot_entry = 0xff0000;
constexpr Word ram_entry = 0x100;
constexpr Word jump_absolute = 0x0af080;
constexpr Word immediate_x0 = 0x44f400;
constexpr unsigned hold_ticks = 90, recovery_ticks = 135;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

template <class Operation> void rejects(Operation operation, const char *message) {
  try {
    operation();
  } catch (const std::logic_error &) { return; } catch (const blofeld::Fault &) {
    return;
  }
  throw std::runtime_error(message);
}

void reset(OwnDspMachine &machine) {
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  require(machine.advance_hardware_reset_ticks(hold_ticks) == hold_ticks, "Reset hold differs");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
          "Reset recovery differs");
}

void read_word(OwnDspFetch &fetch, const Word address) {
  fetch.issue();
  require(fetch.view().pending_address == address, "F1 did not own the expected address");
  fetch.capture();
  require(!fetch.view().pending_address, "F2 did not complete its actual sample");
}

void retire(OwnDspMachine &machine) {
  for (unsigned count = 0; count < 32; ++count) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Native instruction lost its effect"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::none, "Native staged effect failed");
    if (result.retired) { return; }
  }
  throw std::runtime_error("Native instruction exceeded its effect bound");
}

void actual_fetch() {
  OwnDspMachine machine(clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = immediate_x0});
  machine.bus().write({.address = ram_entry + 1, .value = 7});
  reset(machine);
  const auto ticks = machine.clock().elapsed_ticks();
  OwnDspFetch fetch(machine);
  rejects([&] { const OwnDspFetch duplicate(machine); }, "A second fetch owner was accepted");
  rejects([&] { static_cast<void>(machine.begin_fetch()); }, "Competing native fetch was accepted");
  rejects([&] { static_cast<void>(machine.admit_interrupt({.address = boot_entry, .opcode = 0})); },
          "Unqualified interrupt admission was accepted");
  read_word(fetch, boot_entry);
  rejects([&] { static_cast<void>(fetch.begin()); }, "Missing F2 extension was accepted");
  read_word(fetch, boot_entry + 1);
  require(fetch.begin().fault.kind == FaultKind::none, "Real captured jump was rejected");
  rejects([&] { fetch.synchronize(); }, "An unretired instruction was synchronized");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == ram_entry && fetch.view().next_address == ram_entry,
          "Actual jump did not redirect the F1/F2 owner");
  read_word(fetch, ram_entry);
  read_word(fetch, ram_entry + 1);
  machine.bus().write({.address = ram_entry + 1, .value = 11});
  require(fetch.begin().fault.kind == FaultKind::none, "Captured immediate was rejected");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().x == 7 && machine.cpu().registers().pc == ram_entry + 2 &&
              machine.cpu().progress().instructions == 2 &&
              machine.clock().elapsed_ticks() == ticks,
          "CPU must execute the original F2 sample without manufacturing clock time");
  read_word(fetch, ram_entry + 2);
  try {
    static_cast<void>(fetch.begin());
    throw std::runtime_error("Unknown program memory was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("read P:0x000102") && detail.contains("pc=0x000102"),
            "Unknown F2 sample lost its operation, address or PC");
  }
}

void unqualified_cache() {
  constexpr Word movec_sr = 0x05f439;
  constexpr Word cache_status = 0xc80300;
  const std::array<Word, 2> rom{movec_sr, cache_status};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  require(machine.cpu().registers().sr == cache_status, "Native cache status did not retire");
  try {
    fetch.synchronize();
    throw std::runtime_error("Unqualified cache synchronization was accepted");
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("cache ownership"),
            "Cache-dependent F2 must fail with its real cause");
  }
}

void unqualified_repeat() {
  constexpr Word repeat_three = 0x0603a0;
  const std::array<Word, 2> rom{repeat_three, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  try {
    fetch.synchronize();
    throw std::runtime_error("Unqualified REP latch was accepted");
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("REP opcode latch"),
            "REP-dependent F2 must fail with its real cause");
  }
}

void changed_memory_context() {
  constexpr Word movec_omr = 0x05f43a;
  constexpr Word changed_omr = 0x304;
  const std::array<Word, 3> rom{movec_omr, changed_omr, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  fetch.issue();
  static_cast<void>(fetch.begin());
  for (unsigned count = 0; count < 32 && machine.cpu().registers().omr != changed_omr; ++count) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("OMR instruction lost its effect"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::none && !result.retired,
            "OMR change must publish before retirement");
  }
  require(machine.cpu().registers().omr == changed_omr, "OMR transition did not publish");
  try {
    fetch.capture();
    throw std::runtime_error("F2 sampled with a stale memory context");
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("F2 memory context read P:0xff0002"),
            "Stale F2 context lost its actual pending address");
  }
  retire(machine);
  fetch.synchronize();
  require(fetch.view().omr == changed_omr && fetch.view().next_address == boot_entry + 2 &&
              !fetch.view().pending_address,
          "Real OMR retirement must discard the stale F1 and reset its context");
}

void invalidation() {
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  const auto checkpoint = machine.checkpoint();
  OwnDspFetch old(machine);
  read_word(old, boot_entry);
  machine.restore(checkpoint);
  rejects([&] { static_cast<void>(old.view()); }, "Restore retained stale F2 ownership");
  OwnDspFetch fresh(machine);
  read_word(fresh, boot_entry);
  static_cast<void>(fresh.begin());
  reset(machine);
  rejects([&] { fresh.issue(); }, "Hardware reset retained stale F2 ownership");
  OwnDspFetch after_reset(machine);
  read_word(after_reset, boot_entry);
}
}

int main() {
  try {
    actual_fetch();
    invalidation();
    unqualified_cache();
    unqualified_repeat();
    changed_memory_context();
    std::cout << "Own DSP uses actual owned F1/F2 samples.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
