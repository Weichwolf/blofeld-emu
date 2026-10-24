// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::fetch_clock {
namespace {
using native_access::fault;

void unknown_pinit() {
  DspMachine machine;
  DspProgram code;
  code.emit("nop");
  code.install(machine.bus());
  const auto pending = machine.begin_reference_instruction(reference::boundaries);
  const auto before = machine.cpu().debug_state();
  const auto error = fault([&] { const DspFetchRun run(contract(), machine); });
  require(error.contains("X:0xfffffd") && error.contains("pc=0xff0000") &&
              error.contains("omr=0x000305") && machine.cpu().debug_state() == before &&
              reference::ticket(machine) == reference::ticket(pending) &&
              !machine.reference_fetch().active,
          "Unobserved reset PINIT refuses clock ownership before any frontend or native mutation");
}

void invalid_period_recovery() {
  DspMachine machine;
  std::array<std::uint32_t, 16> words{};
  prepare(machine, words);
  auto inputs = contract();
  inputs.timebase_hz = extal;
  DspFetchRun run(inputs, machine);
  static_cast<void>(run.advance_cycle(machine, input()));
  const auto saved = run.checkpoint(machine);
  constexpr unsigned faster = 0x02e112;
  machine.bus().pll().write(faster);
  require(fault([&] {
            static_cast<void>(run.advance_cycle(machine, input()));
          }).contains("Unrepresentable"),
          "A nonintegral reference period fails rather than rounding elapsed time");
  require(run.faulted() && run.elapsed_ticks() == 1 && run.completed_cycles() == 1 &&
              machine.bus().last_dma_cycle() == 1,
          "An impossible period commits no DMA sample, frontend step or clock");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  run.restore(saved, machine);
  const auto recovered = run.advance_cycle(machine, input());
  require(!run.faulted() && recovered.period_ticks == 1 && run.elapsed_ticks() == 2,
          "A joint checkpoint restores the exact prior PLL command and reference period");
}

void owner_and_contract_guards() {
  DspMachine machine;
  DspMachine foreign;
  std::array<std::uint32_t, 16> words{};
  prepare(machine, words);
  prepare(foreign, words);
  auto invalid = contract();
  invalid.timing = DspFetchTiming::unqualified;
  static_cast<void>(fault([&] { const DspFetchRun run(invalid, machine); }));
  DspFetchRun run(contract(), machine);
  DspFetchRun other(contract(), foreign);
  const auto saved = run.checkpoint(machine);
  static_cast<void>(fault([&] { static_cast<void>(run.advance_cycle(foreign, input())); }));
  static_cast<void>(fault([&] { other.restore(saved, foreign); }));
  static_cast<void>(fault([&] { const DspFetchRun duplicate(contract(), machine); }));
  require(run.completed_cycles() == 0 && other.completed_cycles() == 0,
          "Invalid, foreign and duplicate clock ownership cannot advance either real machine");
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 1}), banks::use()));
  require(fault([&] {
            static_cast<void>(run.advance_cycle(machine, input()));
          }).contains("coordinated"),
          "An out-of-band DMA observation cannot create an unobserved clock hole");
  run.restore(saved, machine);
  require(run.advance_cycle(machine, input()).completed_cycles == 1,
          "Trusted joint restore recovers the real prior DMA index and first F1 clock");
}

void completion_owner() {
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = timebase,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  const auto before = machine.cpu().debug_state();
  static_cast<void>(fault([&] { const DspFetchRun run(contract(), machine); }));
  require(!machine.timing_faulted() && machine.cpu().debug_state() == before,
          "The frontend clock cannot replace an existing native completion-clock contract");
  DspProgram code;
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  require(machine.run_clock().native_cycle_units() == 1,
          "Rejected frontend ownership preserves the actual previous nominal path");
}
}

void guard_checks() {
  unknown_pinit();
  invalid_period_recovery();
  owner_and_contract_guards();
  completion_owner();
}
}
