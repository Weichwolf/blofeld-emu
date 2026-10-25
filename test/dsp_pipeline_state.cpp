// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pipeline.hpp"
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

void joint_replay_and_captured_code() {
  DspMachine machine;
  std::array<const char *, 16> code{};
  code.fill("add x1,a");
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto saved = run.checkpoint(machine);
  const auto old_ticket = reference::ticket(machine);
  const auto initial_view = run.pipeline();
  require(initial_view && initial_view->awaiting_retirement &&
              initial_view->awaiting_retirement->address == body,
          "E1 has captured the first actual native result before E2 publication");
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    static_cast<void>(
        fault([&] { static_cast<void>(machine.advance_reference_access(old_ticket)); }));
    machine.bus().write({.address = body});
    machine.bus().write({.address = body + 1});
    const auto first = run.advance_cycle(machine, fetch_clock::input());
    require(machine.cpu().state().a == unit && stages(first).execution_captured,
            "Actual old opcode replacement cannot change the completed E1 candidate");
    const auto second = run.advance_cycle(machine, fetch_clock::input());
    require(machine.cpu().state().a == 2 * unit && stages(second).retirement_published,
            "A younger opcode replaced after F2 still executes its real captured original word");
    const auto view = run.pipeline();
    if (!view) { throw Fault("The restored automatic stage state is missing"); }
    const auto decoded = view->stages[0];
    const auto held = view->awaiting_retirement;
    require(decoded && decoded->address == body + 5 && held && held->address == body + 2,
            "Joint replay retains decode/AG latches and the newer actual E1 result together");
  }
}

void future_code_is_really_fetched() {
  DspMachine machine;
  std::array<const char *, 16> code{};
  code.fill("add x1,a");
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto replacement = checkpoint::assemble("clr a");
  require(replacement.size() == 1, "Own replacement is one genuine ALU word");
  machine.bus().write({.address = body + 15, .value = replacement.front()});
  for (unsigned retired = 1; retired <= 16; ++retired) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    const auto value = retired == 16 ? 0 : retired * unit;
    require(machine.cpu().state().a == value,
            "A replacement before actual F2 changes only the corresponding future native result");
  }
}

void failed_f2_joint_recovery() {
  DspMachine machine;
  constexpr std::array code{"add x1,a"};
  prepare(machine, code, {.padding = 0});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 2);
  const auto saved = run.checkpoint(machine);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(
      error.contains("P:") && error.contains("completed-cycles=2") && run.faulted() &&
          run.completed_cycles() == 2 && run.elapsed_ticks() == 2 * period &&
          machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 3,
      "Failed actual F2 retains the entered DMA prefix without advancing decode or elapsed time");
  const auto failed = run.pipeline();
  require(failed && failed->cycle.decode && !failed->cycle.advanced && !failed->stages[0],
          "The planned decode remains unapplied after the real F2 fault");
  run.restore(saved, machine);
  for (unsigned index = 1; index <= 16; ++index) { machine.bus().write({.address = body + index}); }
  clocks(run, machine, 5);
  require(!run.faulted() && machine.cpu().state().a == unit && run.completed_cycles() == 7,
          "Trusted joint recovery restores native/frontend and automatic stage clocks together");
}
}

void state_checks() {
  joint_replay_and_captured_code();
  future_code_is_really_fetched();
  failed_f2_joint_recovery();
}
}
