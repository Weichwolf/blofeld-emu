// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "lcd_fixture.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "original_owned_clock_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <string>

namespace blofeld::test::owned_clock {
namespace {
constexpr unsigned continuation_limit = 20000000;
constexpr unsigned continuation_steps = 1570275;
constexpr unsigned continuation_retired = 1570072;
constexpr unsigned continuation_entries = 203;
constexpr unsigned extra_flash_words = 70;
constexpr unsigned display_commands = 344;
constexpr unsigned display_writes = 4194;
constexpr unsigned display_reads = 2244;
constexpr unsigned wai_opcode = 0x3e;
constexpr unsigned irq_enabled = 0x80;
constexpr unsigned first_wait_chunk = 1000;
constexpr unsigned remaining_wait = 1282;
constexpr unsigned total_wait = 2282;
constexpr unsigned wait_limit = 10000;
constexpr unsigned oscillator_remaining = 913;
constexpr std::uint64_t pending_cycles = 141812081;
constexpr std::uint64_t pending_ticks = 283624826;
constexpr std::uint64_t pending_oscillator = 56724965;

void check_frame(McuMachine &machine, const CpuState &before) {
  const auto state = machine.cpu().state();
  require(before.pc == main_wait && before.sp == main_stack &&
              (before.ccr & hc12::interrupt_mask) == 0 && state.pc == main_resume &&
              state.sp == wait_stack && state.d == before.d && state.x == before.x &&
              state.y == before.y && state.ccr == before.ccr && machine.cpu().waiting(),
          "Original main WAI preserves the native registers and interrupt-enable state");
  const std::array<unsigned, 9> frame{before.ccr,
                                      before.d & byte_mask,
                                      static_cast<unsigned>(before.d) >> 8U,
                                      static_cast<unsigned>(before.x) >> 8U,
                                      before.x & byte_mask,
                                      static_cast<unsigned>(before.y) >> 8U,
                                      before.y & byte_mask,
                                      main_resume >> 8U,
                                      main_resume & byte_mask};
  for (unsigned index = 0; index < frame.size(); ++index) {
    require(machine.bus().read(static_cast<std::uint16_t>(wait_stack + index)) == frame[index],
            "Original main WAI frame agrees with the independent NXP byte layout");
  }
}

void record_section(const std::string &section,
                    const unsigned level,
                    const Census &census,
                    const Interrupts &interrupts,
                    const FlashPhases &phases,
                    McuMachine &machine,
                    std::ostream &trace) {
  trace << section << " input=" << level << " retired=" << census.retired
        << " entries=" << interrupts.entries << " reused=" << interrupts.reused
        << " cycles=" << machine.run_clock().bus_cycles() << " idle=" << machine.idle_bus_cycles()
        << " ticks=" << machine.run_clock().elapsed_ticks()
        << " OSC=" << machine.run_clock().oscillator_cycles() << " extra-flash=" << phases.low
        << " pc=" << hex(machine.cpu().state().pc) << " sp=" << hex(machine.cpu().state().sp)
        << '\n';
  trace.flush();
}

CpuState run_to_main(McuMachine &machine,
                     LcdFixture &display,
                     const std::filesystem::path &path,
                     std::ostream &trace,
                     const unsigned level) {
  Census census;
  FlashPhases phases;
  Inputs inputs{.phy = true, .capture = true};
  Interrupts interrupts;
  attach(machine, display, phases, trace);
  CpuState before;
  unsigned steps = 0;
  try {
    for (; steps < continuation_limit && !machine.cpu().waiting(); ++steps) {
      before = machine.cpu().state();
      inputs.observe(machine);
      census.before(machine);
      const auto opcode = machine.bus().peek(before.pc);
      const auto result = machine.cpu().step();
      census.retire(before, result);
      interrupts.observe(before, result, machine.cpu(), opcode);
    }
  } catch (...) {
    census.write(artifact(path, ".census.jsonl"));
    phases.write(artifact(path, ".phases.json"));
    throw;
  }
  census.write(artifact(path, ".census.jsonl"));
  phases.write(artifact(path, ".phases.json"));
  record_section("main", level, census, interrupts, phases, machine, trace);
  require(steps == continuation_steps && census.retired == continuation_retired &&
              interrupts.entries == continuation_entries &&
              interrupts.returns == continuation_entries && phases.high == extra_flash_words &&
              phases.low == extra_flash_words && !phases.pending &&
              machine.run_clock().bus_cycles() == main_cycles &&
              machine.run_clock().elapsed_ticks() == main_ticks &&
              machine.run_clock().oscillator_cycles() == main_oscillator &&
              machine.idle_bus_cycles() == 0 && !machine.cpu().pending_interrupt() &&
              machine.bus().read(saved_input) == level &&
              machine.bus().peek(before.pc) == wai_opcode,
          "Original clocked continuation must reach its exact first main WAI/input/time frontier");
  check_frame(machine, before);
  const auto counts = display.counts();
  require(counts.commands == display_commands && counts.writes == display_writes &&
              counts.reads == display_reads,
          "Original clocked startup retains independently pinned display transaction counts");
  detach(machine);
  return before;
}

void reach_pending_interrupt(McuMachine &machine) {
  require(machine.advance_wait(wait_limit) == remaining_wait &&
              machine.idle_bus_cycles() == total_wait &&
              machine.run_clock().bus_cycles() == pending_cycles &&
              machine.run_clock().elapsed_ticks() == pending_ticks &&
              machine.run_clock().oscillator_cycles() == pending_oscillator,
          "The original RTICTL write and native source ledger independently predict WAI timeout");
  const auto pending = machine.cpu().pending_interrupt();
  require(pending && pending->vector == realtime_vector && machine.advance_wait(wait_limit) == 0,
          "Idle time stops exactly at the first real RTI without executing or overshooting it");
}

void run_wake(McuMachine &machine,
              LcdFixture &display,
              const CpuState &first_before,
              const std::filesystem::path &path,
              std::ostream &trace,
              const unsigned level) {
  Census census;
  FlashPhases phases;
  Inputs inputs{.phy = true, .capture = true};
  auto resumed = first_before;
  resumed.pc = main_resume;
  Interrupts interrupts{.foreground = resumed, .entries = 1, .reused = 1};
  attach(machine, display, phases, trace);
  reach_pending_interrupt(machine);
  const auto entry = machine.cpu().step();
  require(!entry.instruction_executed && entry.interrupt_vector == realtime_vector &&
              entry.stack_reused && machine.cpu().state().sp == wait_stack &&
              machine.cpu().state().pc == realtime_handler &&
              machine.run_clock().bus_cycles() == pending_cycles + 6,
          "Original RTI wakes WAI with the native six-cycle fVfPPP sequence and retained frame");
  CpuState before;
  unsigned steps = 0;
  try {
    for (; steps < continuation_limit && !machine.cpu().waiting(); ++steps) {
      before = machine.cpu().state();
      inputs.observe(machine);
      census.before(machine);
      const auto opcode = machine.bus().peek(before.pc);
      const auto result = machine.cpu().step();
      census.retire(before, result);
      interrupts.observe(before, result, machine.cpu(), opcode);
    }
  } catch (...) {
    census.write(artifact(path, ".census.jsonl"));
    phases.write(artifact(path, ".phases.json"));
    throw;
  }
  census.write(artifact(path, ".census.jsonl"));
  phases.write(artifact(path, ".phases.json"));
  record_section("wake", level, census, interrupts, phases, machine, trace);
  require(
      interrupts.returns == interrupts.entries && !interrupts.foreground &&
          !machine.cpu().pending_interrupt() && !machine.timing_faulted() &&
          (machine.bus().read(hc12::crg_flags) & irq_enabled) == 0 &&
          machine.bus().read(saved_input) == level && !phases.pending && phases.high == phases.low,
      "Original wake must acknowledge RTIF, restore foreground and return naturally to main WAI");
  check_frame(machine, before);
}

void verify_idle_replay(McuMachine &machine,
                        LcdFixture &display,
                        const CpuState &before,
                        const std::filesystem::path &path,
                        std::ostream &trace,
                        const unsigned level) {
  const auto gates = machine.bus().wait_clock_gates();
  require(
      gates.sci && gates.timer && machine.bus().read(hc12::clock_select) == irq_enabled &&
          machine.bus().oscillator_cycles_until_rti() == oscillator_remaining,
      "Actual original wait configuration continues system/SCI/TIM clocks with known RTI phase");
  require(machine.advance_wait(first_wait_chunk) == first_wait_chunk &&
              !machine.cpu().pending_interrupt(),
          "The first partial original WAI slice stops before the independently predicted IRQ");
  const auto frozen = machine.checkpoint();
  const LcdFixture frozen_display(machine.bus(), display);
  const auto initial = machine.bus().power_off_flash_image();
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    machine.restore(frozen);
    LcdFixture restored_display(machine.bus(), frozen_display);
    const auto replay_path = artifact(path, attempt == 0 ? ".wake0" : ".wake1");
    run_wake(machine, restored_display, before, replay_path, trace, level);
    check_nvm(machine, initial);
    detach(machine);
  }
}
}

void run_continuations(McuMachine &machine,
                       const PrefixCheckpoint &prefix,
                       const std::filesystem::path &path,
                       std::ostream &trace) {
  for (const unsigned level : {0U, 8U}) {
    machine.restore(prefix.machine);
    require(machine.run_clock().bus_cycles() == prefix_cycles && !machine.timing_faulted(),
            "Coordinated strict-input recovery restores the exact pre-read clock boundary");
    LcdFixture display(machine.bus(), prefix.display);
    machine.bus().set_pin_drive(McuPort::m,
                                {.mask = 8, .levels = static_cast<std::uint8_t>(level)});
    const auto case_path = artifact(path, level == 0 ? ".low" : ".high");
    const auto initial = machine.bus().power_off_flash_image();
    const auto before = run_to_main(machine, display, case_path, trace, level);
    check_nvm(machine, initial);
    verify_midi_reference_replays(machine, display, case_path, trace, level);
    verify_idle_replay(machine, display, before, case_path, trace, level);
    std::cout << "Original clocked input=" << level
              << ": main WAI and native idle/RTI replay passed." << '\n';
    std::cout.flush();
  }
}
}

namespace {
struct Arguments {
  std::filesystem::path reference;
  std::filesystem::path trace;
};

void run(const Arguments &arguments) {
  const auto firmware = blofeld::Firmware::load(arguments.reference);
  constexpr unsigned oscillator_hz = 12000000;
  blofeld::McuMachine machine(
      blofeld::McuFlashImage::from_known(firmware.mcu),
      BLOFELD_DEFAULT_SLA,
      blofeld::McuRunContract{
          .clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads});
  std::ofstream trace(arguments.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  const auto prefix = blofeld::test::owned_clock::run_prefix(machine, arguments.trace, trace);
  blofeld::test::owned_clock::run_continuations(machine, prefix, arguments.trace, trace);
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-owned-clock firmware.zip-or-mid private.trace");
    }
    run({.reference = arguments[1], .trace = arguments[2]});
    std::cout << "Conditional original MCU reset/main WAI, native MIDI/RTI and trusted peer replay "
                 "passed. Full DSP/board/audio and physical timing remain unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
