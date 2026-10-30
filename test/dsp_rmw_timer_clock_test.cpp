// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, extal = 12000000, timebase = 1080000000;
constexpr std::uint64_t reset_ticks = 180;
constexpr unsigned padding = 16, words = 33, before_read = 21, after_read = 22;
// FM13-36: BSET #0,X:$ffff8f. DSP56371UM TCSR: TCF is W1C, TE is bit0.
constexpr unsigned set_enable = 0x010f20, control = 0xffff8f, compare_flag = 0x200000;
// First CLK/2 edge loads0; edge11 compares10, after this program's E1 bus read.
constexpr unsigned compare = 10, after_read_edges = 11;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

DspFetchCycleInputs inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

void finish(DspMachine &machine, DspFetchRun &run, const DspDebugState &before) {
  auto expected = before;
  ++expected.cpu.pc;
  expected.cpu.sr |= 1;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  const auto cycle = run.advance_cycle(machine, inputs());
  require(cycle.bus.bit_write && cycle.bus.bit_write->value == 1 && cycle.pipeline &&
              cycle.pipeline->retirement_published && machine.cpu().debug_state() == expected &&
              run.completed_cycles() == after_read + 1 &&
              run.timer_clk2_edges() == after_read_edges &&
              machine.bus().timers().read(control, compare_flag | 1) == (compare_flag | 1) &&
              machine.bus().timers().read(control - 3) == compare,
          "E2 writes the captured pre-compare word, retaining the genuine newer W1C flag and exact "
          "CPU/clock state");
}

void execute() {
  DspMachine machine;
  std::array<std::uint32_t, words> program{};
  program.at(padding) = set_enable;
  machine.bus().install_rom({.start = boot, .words = program});
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "The actual CPU is held in reset for two EXTAL periods");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "The actual CPU completes two EXTAL periods of reset recovery");
  machine.bus().timers().write({.address = control - 1, .value = 0});
  machine.bus().timers().write({.address = control - 2, .value = compare});
  machine.bus().timers().write({.address = control, .value = 1});
  for (unsigned i = 0; i < before_read; ++i) {
    static_cast<void>(run.advance_cycle(machine, inputs()));
  }
  auto before = machine.cpu().debug_state();
  require(before.cpu.pc == boot + padding - 1 && before.native_instructions == padding - 1 &&
              run.timer_clk2_edges() == after_read_edges - 1 &&
              machine.bus().timers().read(control, compare_flag) == 0 &&
              machine.bus().timers().read(control - 3) == compare - 1,
          "Literal NOP prefix reaches the actual RMW before its eleventh CLK/2 edge, with no flag");
  // The older final NOP retires in the same period as the younger bit's E1.
  ++before.cpu.pc;
  ++before.native_instructions;
  ++before.native_cycles;
  const auto read = run.advance_cycle(machine, inputs());
  require(
      read.bus.memory && read.bus.memory->bit_read && machine.cpu().debug_state() == before &&
          run.completed_cycles() == after_read && run.timer_clk2_edges() == after_read_edges &&
          machine.bus().timers().read(control, compare_flag) == compare_flag &&
          machine.bus().timers().read(control - 3) == compare,
      "The fetch owner's real CLK/2 edge sets TCF after the actual E1 read, before any E2 write");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 3; ++replay) {
    run.restore(saved, machine);
    finish(machine, run, before);
  }
  DspMachine target;
  DspFetchRun cloned(saved, target);
  finish(target, cloned, before);
  std::cout << "PASS actual CLK/2 post-read compare;E1=22,E2=23,edges=11;restores=3,clone=1\n";
}
}

int main() {
  try {
    execute();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL native clocked RMW timer: " << error.what() << '\n';
    return 1;
  }
}
