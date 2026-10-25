// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>

namespace blofeld::test::pipeline {
namespace {
using native_access::fault;

void phase_and_contract_guards() {
  DspMachine machine;
  constexpr std::array code{"add x1,a"};
  prepare(machine, code);
  auto bad = contract();
  bad.execution = reference::boundaries;
  const auto before = machine.cpu().debug_state();
  static_cast<void>(fault([&] { const DspFetchRun invalid(bad, machine); }));
  require(!machine.reference_fetch().active && machine.cpu().debug_state() == before,
          "An incompatible automatic result contract refuses before starting the frontend");
  DspFetchRun run(contract(), machine);
  static_cast<void>(
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input(true))); }));
  static_cast<void>(fault([&] { static_cast<void>(run.begin_instruction(machine)); }));
  static_cast<void>(fault([&] { static_cast<void>(run.overlap_instruction(machine)); }));
  require(!run.faulted() && run.completed_cycles() == 0 && machine.cpu().debug_state() == before,
          "Manual native phase overrides refuse without changing automatic pipeline ownership");
  clocks(run, machine, 7);
  require(machine.cpu().state().a == unit,
          "The automatic clock remains usable after pure API refusals");
}

void decode_refuses_before_fifo() {
  DspMachine machine;
  constexpr std::array code{"move x:(r0)+,x0"};
  prepare(machine, code, {.pointer = reference::fifo});
  // A valid pending SHI receive must survive the unsupported memory-stage decode.
  constexpr unsigned control = 0xffff91;
  constexpr unsigned configuration = 0xa9;
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = control, .value = configuration});
  reference::send(machine.bus().shi(), reference::first);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 1);
  const auto before_f2 = run.checkpoint(machine);
  clocks(run, machine, 1);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(
      error.contains("automatic DSP decode P:") && error.contains("pc=") &&
          error.contains("omr=") && run.faulted() && run.completed_cycles() == 2 &&
          machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 2 &&
          machine.bus().read({.space = DspSpace::x, .address = reference::fifo}) ==
              reference::first,
      "Unqualified load placement fails with decode context before native pointer or data effects");
  run.restore(before_f2, machine);
  const auto replacement = checkpoint::assemble("add x1,a");
  machine.bus().write({.address = body, .value = replacement.front()});
  clocks(run, machine, 6);
  require(machine.cpu().state().a == unit,
          "Repair before the actual replayed F2, then joint recovery, admits the real replacement");
}

void mode_and_dma_guards() {
  constexpr std::array code{"nop"};
  for (const unsigned status : {masked_status | 0x80000U, masked_status | 0x20000U}) {
    DspMachine machine;
    prepare(machine, code, {.status = status});
    DspFetchRun run(contract(), machine);
    const auto before = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(
        error.contains("mode/interrupt") && machine.cpu().debug_state() == before &&
            run.completed_cycles() == 0 && machine.bus().last_dma_cycle() == 0,
        "Actual cache/16-bit arithmetic modes refuse before an unqualified automatic stage clock");
  }
  DspMachine machine;
  prepare(machine, code, {.dma = true});
  DspFetchRun run(contract(), machine);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("X:0xfffff4") && error.contains("pc=") &&
              machine.cpu().debug_state() == before && run.completed_cycles() == 0 &&
              machine.bus().last_dma_cycle() == 0,
          "Active DMA is a visible automatic-timing frontier; explicit reference arbitration stays "
          "separate");
}

void real_interrupt_is_not_skipped() {
  DspMachine machine;
  constexpr std::array code{"nop"};
  prepare(machine, code, {.status = checkpoint::ordinary_status});
  DspFetchRun run(contract(), machine);
  for (const auto store :
       {DspStore{.space = DspSpace::x, .address = fetch_clock::csr0},
        DspStore{.space = DspSpace::x, .address = checkpoint::iprp, .value = 0x100},
        DspStore{.space = DspSpace::x, .address = fetch_clock::load0, .value = 1},
        DspStore{.space = DspSpace::x, .address = fetch_clock::compare0, .value = 1},
        DspStore{.space = DspSpace::x, .address = fetch_clock::csr0, .value = 7}}) {
    machine.bus().write(store);
  }
  machine.bus().timers().advance_clk2(1);
  require(machine.bus().interrupt_request(0).has_value(), "An actual timer IRQ is pending");
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("mode/interrupt") && run.completed_cycles() == 0 &&
              machine.bus().interrupt_request(0).has_value(),
          "Unqualified automatic interrupt timing refuses without inventing acknowledgement or "
          "skipping it");
}
}

void guard_checks() {
  phase_and_contract_guards();
  decode_refuses_before_fifo();
  mode_and_dma_guards();
  real_interrupt_is_not_skipped();
}
}
