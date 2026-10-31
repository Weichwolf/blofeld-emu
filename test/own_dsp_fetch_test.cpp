// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <initializer_list>
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
constexpr Word movec_sr = 0x05f439;
constexpr Word irqa_vector = 0x10;
constexpr Word priority_register = 0xffffff;
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

void vector_read(OwnDspMachine &machine, const Word expected, const bool acknowledged) {
  const auto pending = machine.cpu().pending_instruction();
  if (!pending) { throw std::runtime_error("Native vector lost its fetch"); }
  require(pending->effect.kind == EffectKind::fetch_opcode && pending->effect.address == expected &&
              pending->effect.pc == expected && pending->effect.space == Space::program,
          "Native vector must own the actual program-bus access");
  const auto result = machine.advance(pending->ticket);
  require(result.fault.kind == FaultKind::none &&
              machine.bus().interrupt_request(0).has_value() != acknowledged,
          "Interrupt acknowledgement must follow the second actual vector read");
}

void interrupt_vector(const bool missing_second) {
  const std::array<Word, 4> rom{movec_sr, 0, immediate_x0, 7};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector, .value = 0});
  if (!missing_second) { machine.bus().write({.address = irqa_vector + 1, .value = 0}); }
  reset(machine);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  require(!fetch.admit_interrupt() && machine.bus().interrupt_request(0).has_value(),
          "Masked interrupt must remain pending with the actual F2 foreground");
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  read_word(fetch, boot_entry + 3);
  require(fetch.admit_interrupt(), "Unmasked real IRQA was not admitted");
  rejects([&] { fetch.issue(); }, "F2 prefetch was accepted during native vector ownership");
  static_cast<void>(fetch.begin());
  vector_read(machine, irqa_vector, false);
  retire(machine);
  fetch.synchronize();
  static_cast<void>(fetch.begin());
  if (missing_second) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Second vector read is missing"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
                result.fault.address == irqa_vector + 1 && result.fault.pc == irqa_vector + 1 &&
                machine.io().failure() && machine.bus().interrupt_request(0).has_value(),
            "Unknown vector word must fail with its actual address and retain IRQ ownership");
    return;
  }
  vector_read(machine, irqa_vector + 1, true);
  retire(machine);
  fetch.synchronize();
  require(fetch.view().next_address == boot_entry + 2 &&
              machine.cpu().registers().pc == boot_entry + 2,
          "Fast vector must resume the actual foreground PC");
  read_word(fetch, boot_entry + 2);
  read_word(fetch, boot_entry + 3);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().x == 7, "Foreground must execute after the actual vector");
}

void interrupt_call(const bool short_vector) {
  constexpr Word long_call = 0x0bf080;
  constexpr Word short_call = 0x0d0011;
  constexpr Word handler = 0x200;
  const std::array<Word, 3> rom{movec_sr, 0, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector, .value = short_vector ? short_call : long_call});
  machine.bus().write({.address = irqa_vector + 1, .value = handler});
  machine.bus().write({.address = handler, .value = 0});
  reset(machine);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  require(fetch.admit_interrupt(), "Real IRQA call was not admitted");
  static_cast<void>(fetch.begin());
  vector_read(machine, irqa_vector, false);
  if (short_vector) {
    retire(machine);
    require(machine.cpu().registers().pc == irqa_vector + 1 && machine.bus().interrupt_request(0),
            "Short vector must retain its actual unacknowledged source");
    try {
      fetch.synchronize();
      throw std::runtime_error("Unqualified one-word vector was accepted");
    } catch (const blofeld::Fault &error) {
      require(std::string(error.what()).contains("interrupt vector continuation"),
              "Unqualified vector must report its real continuation");
    }
    return;
  }
  const auto extension = machine.cpu().pending_instruction();
  if (!extension) { throw std::runtime_error("Long vector extension is missing"); }
  require(extension->effect.kind == EffectKind::fetch_extension &&
              extension->effect.address == irqa_vector + 1,
          "Long vector must read the actual second vector word");
  const auto result = machine.advance(extension->ticket);
  require(result.fault.kind == FaultKind::none && !machine.bus().interrupt_request(0),
          "Real second vector word must acknowledge the source");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == handler && machine.cpu().registers().sp == 1 &&
              fetch.view().next_address == handler,
          "Long interrupt call must install its real frame and handler fetch");
  read_word(fetch, handler);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == handler + 1,
          "The native interrupt handler must execute its actual instruction");
}

void missing_first_vector() {
  const std::array<Word, 3> rom{movec_sr, 0, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector + 1, .value = 0});
  reset(machine);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  require(fetch.admit_interrupt(), "Real IRQ with unknown vector was not admitted");
  static_cast<void>(fetch.begin());
  const auto pending = machine.cpu().pending_instruction();
  if (!pending) { throw std::runtime_error("First vector read is missing"); }
  const auto result = machine.advance(pending->ticket);
  require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
              result.fault.address == irqa_vector && result.fault.pc == irqa_vector &&
              machine.io().failure() && machine.bus().interrupt_request(0).has_value(),
          "Unknown first vector must retain IRQ and report its true read and PC");
}

void software_vector() {
  constexpr Word trap = 6;
  constexpr Word trap_vector = 8;
  const std::array<Word, 2> rom{trap, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = trap_vector, .value = 0});
  machine.bus().write({.address = trap_vector + 1, .value = 0});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().instruction_fetch().exception_vector &&
              fetch.view().next_address == trap_vector,
          "The native TRAP must redirect to its actual software vector");
  for (const auto address : {trap_vector, trap_vector + 1}) {
    read_word(fetch, address);
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
  }
  require(!machine.cpu().instruction_fetch().exception_vector &&
              machine.cpu().registers().pc == boot_entry + 1 &&
              fetch.view().next_address == boot_entry + 1,
          "Software-vector completion must return to the actual foreground");
}

void repeat_latch() {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word add_x0 = 0x200040;
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = immediate_x0});
  machine.bus().write({.address = ram_entry + 1, .value = 7});
  machine.bus().write({.address = ram_entry + 2, .value = repeat_three});
  machine.bus().write({.address = ram_entry + 3, .value = add_x0});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, ram_entry);
  read_word(fetch, ram_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, ram_entry + 2);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, ram_entry + 3);
  machine.bus().write({.address = ram_entry + 3, .value = 0});
  static_cast<void>(fetch.begin());
  retire(machine);
  rejects([&] { fetch.issue(); }, "F1 after retirement bypassed REP synchronization");
  fetch.synchronize();
  for (unsigned iteration = 0; iteration < 2; ++iteration) {
    require(machine.cpu().instruction_fetch().latched_opcode == add_x0 &&
                fetch.view().consume_address == ram_entry + 4,
            "REP must own the actual original F2 opcode latch");
    rejects([&] { fetch.issue(); }, "Unqualified REP prefetch was accepted");
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
  }
  constexpr unsigned fractional_shift = 24;
  constexpr std::uint64_t expected_sum = 21;
  require(machine.cpu().registers().a == (expected_sum << fractional_shift) &&
              machine.cpu().registers().pc == ram_entry + 4 &&
              machine.cpu().progress().instructions == 6 && fetch.view().used == 0,
          "All REP iterations must execute the genuine latch despite overwritten RAM");
}

void missing_repeat_provenance() {
  constexpr Word repeat_three = 0x0603a0;
  const std::array<Word, 2> rom{repeat_three, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  {
    OwnDspFetch fetch(machine);
    read_word(fetch, boot_entry);
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
    read_word(fetch, boot_entry + 1);
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
  }
  require(machine.cpu().instruction_fetch().latched_opcode == 0,
          "Native REP must still own the original NOP latch");
  rejects([&] { const OwnDspFetch unproven(machine); },
          "A new F2 owner fabricated provenance for a live REP latch");
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  require(machine.cpu().progress().instructions == 3,
          "Rejected F2 ownership must preserve the actual native REP continuation");
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
    repeat_latch();
    missing_repeat_provenance();
    changed_memory_context();
    interrupt_vector(false);
    interrupt_vector(true);
    interrupt_call(false);
    interrupt_call(true);
    missing_first_vector();
    software_vector();
    std::cout << "Own DSP uses actual owned F1/F2 samples.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
