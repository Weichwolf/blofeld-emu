// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
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

void unknown_decode_joint_recovery() {
  DspMachine machine;
  constexpr std::array code{"add x1,a"};
  prepare(machine, code, {.padding = 0});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 2);
  const auto saved = run.checkpoint(machine);
  const auto before = machine.cpu().debug_state();
  const auto accepted = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = stages(accepted);
  const auto fetched = machine.reference_fetch();
  require(phase.decode_accepted && phase.decode && phase.decode->address == body &&
              fetched.used == 2 && fetched.words[1].address == body + 1 && !fetched.words[1].data &&
              run.completed_cycles() == 3 && run.elapsed_ticks() == 3 * period &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 3,
          "Mapped unknown F2 retains absence while the older known ADD enters Decode");
  const auto unresolved = run.checkpoint(machine);
  for (const unsigned replacement : {0U, 0xffffffU}) {
    run.restore(unresolved, machine);
    machine.bus().write({.address = body + 1, .value = replacement});
    require(machine.reference_fetch() == fetched,
            "A later RAM write cannot initialize an already captured unknown F2 receipt");
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("Unknown DSP program sample read P:" + hex(body + 1, 6)) &&
                error.contains("pc=") && error.contains("omr=") &&
                error.contains("completed-cycles=3") && run.faulted() &&
                run.completed_cycles() == 3 && run.elapsed_ticks() == 3 * period &&
                machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 3,
            "Actual unknown Decode fails before another DMA phase, CPU effect or elapsed clock");
  }
  run.restore(saved, machine);
  for (unsigned index = 1; index <= 16; ++index) { machine.bus().write({.address = body + index}); }
  clocks(run, machine, 5);
  auto expected = before;
  expected.cpu.a = unit;
  expected.cpu.pc = body + 1;
  expected.cpu.sr = masked_status | unnormalized;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(!run.faulted() && machine.cpu().debug_state() == expected &&
              run.completed_cycles() == 7 && run.elapsed_ticks() == 7 * period &&
              machine.bus().last_dma_cycle() == 7,
          "Restore before F2 admits fresh known samples and genuine complete ADD retirement");
}
}

void state_checks() {
  joint_replay_and_captured_code();
  future_code_is_really_fetched();
  unknown_decode_joint_recovery();
}
}
