// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_overlap_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <cstdint>
#include <variant>
#include <vector>

namespace blofeld::test::overlap {
namespace {
namespace clock = fetch_clock;
using native_access::fault;
constexpr unsigned body = clock::program + 4;
constexpr std::uint64_t period = 45;

DspFetchRunInputs contract() {
  auto result = clock::contract();
  result.execution = execution;
  return result;
}

DspFetchCycleInputs both() {
  auto result = clock::input(true);
  result.retire_previous = true;
  return result;
}

void prepare(DspMachine &machine, const bool dma = false, const unsigned padding = 16) {
  auto program = checkpoint::assemble("move #>$400000,x0");
  for (const auto *const text : {"move #>$400000,y0", "mpy y0,x0,a", "move x:(r0)+,x0"}) {
    const auto words = checkpoint::assemble(text);
    program.insert(program.end(), words.begin(), words.end());
  }
  program.insert(program.end(), padding, 0);
  clock::prepare(machine,
                 program,
                 {.transfer = {.source = clock::program_source,
                               .destination = body,
                               .control = dma ? clock::program_copy : 0},
                  .execution = execution});
  for (unsigned count = 0; count < 2; ++count) {
    const auto progress = await_retirement(machine, {.pending = machine.reference_pending()});
    require(machine.advance_reference_access(reference::ticket(progress)).retired,
            "Real setup immediates retire before the two-candidate clock epoch");
    static_cast<void>(machine.begin_reference_instruction(execution));
  }
}

void capture_both(DspFetchRun &run, DspMachine &machine) {
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  static_cast<void>(run.overlap_instruction(machine));
  for (unsigned cycle = 0; cycle < 2; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  require(machine.cpu().state().a == 0 && machine.cpu().state().pc == body &&
              machine.cpu().state().r[0] == clock::initial_r0 + 1,
          "The actual younger R update precedes either retirement and actual data read");
}

void shared_clock_publication() {
  DspMachine machine;
  prepare(machine);
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  const auto count = machine.cpu().debug_state().native_instructions;
  DspFetchRun run(contract(), machine);
  const auto initial = machine.cpu().debug_state();
  static_cast<void>(fault([&] { static_cast<void>(run.overlap_instruction(machine)); }));
  require(!run.faulted() && run.completed_cycles() == 0 && machine.cpu().debug_state() == initial,
          "An invalid handoff refuses before changing clock, CPU or ownership");
  capture_both(run, machine);
  static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  const auto before_invalid = machine.cpu().debug_state();
  const auto dma_before = machine.bus().last_dma_cycle();
  DspReferenceCycleInputs invalid;
  invalid.dma.cycle = 7;
  invalid.order = DspReferenceBusOrder::dma_then_fetch_then_native;
  invalid.retirement = reference::ticket(machine);
  const auto invalid_error =
      fault([&] { static_cast<void>(machine.advance_reference_bus_cycle(invalid)); });
  require(invalid_error.contains("previous native retirement ticket") &&
              machine.bus().last_dma_cycle() == dma_before &&
              machine.cpu().debug_state() == before_invalid && !run.faulted(),
          "A crossed combined-cycle role refuses before any DMA, clock or CPU mutation");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    machine.bus().write(
        {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::third});
    const auto result = run.advance_cycle(machine, both());
    require(result.bus.retirement && result.bus.retirement->pc == body && result.bus.progress &&
                result.bus.progress->completed &&
                std::holds_alternative<DspMoveWriteback>(*result.bus.progress->completed) &&
                result.bus.core_access_count == 1 && run.elapsed_ticks() == 7 * period &&
                machine.cpu().state().a == quarter && machine.cpu().state().x == reference::first &&
                machine.cpu().state().pc == body + 1 &&
                machine.cpu().debug_state().native_instructions == count + 1,
            "One selected clock publishes older E2 and younger E1 with one actual concurrent F2");
    const auto after_old = run.checkpoint(machine);
    const auto final = run.advance_cycle(machine, clock::input(true));
    require(final.bus.progress && final.bus.progress->retired &&
                machine.cpu().state().pc == body + 2 && machine.cpu().state().a == quarter &&
                machine.cpu().state().x == reference::first &&
                machine.cpu().debug_state().native_instructions == count + 2,
            "Later younger retirement retains both captured results and admits no second read");
    run.restore(after_old, machine);
    const auto repeated = run.advance_cycle(machine, clock::input(true));
    require(repeated.bus.progress && repeated.bus.progress->retired,
            "The joint checkpoint after older retirement retains the younger result");
  }
}

void dma_holds_both() {
  DspMachine machine;
  prepare(machine, true);
  machine.bus().write({.address = clock::program_source});
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  DspFetchRun run(contract(), machine);
  capture_both(run, machine);
  auto input = clock::input(true);
  input.requests = 1;
  const auto read = run.advance_cycle(machine, input);
  require(read.bus.dma.phase && read.bus.dma.phase->kind == DspDmaPhaseKind::source_read,
          "Actual program DMA begins while the younger real data load is captured");
  const auto visible = machine.cpu().debug_state();
  const auto saved = run.checkpoint(machine);
  input = both();
  input.grant = DspDmaGrant::wait;
  const auto wait = run.advance_cycle(machine, input);
  require(wait.bus.dma.core_stall && !wait.bus.retirement && !wait.bus.progress &&
              machine.cpu().debug_state() == visible,
          "A real DMA wait holds both actual candidates without publishing either result");
  input.grant = DspDmaGrant::available;
  const auto write = run.advance_cycle(machine, input);
  require(write.bus.dma.word && write.bus.dma.core_stall && !write.bus.retirement &&
              machine.cpu().debug_state() == visible && machine.bus().read({.address = body}) == 0,
          "DMA overwrites the captured older opcode while both candidates remain held");
  const auto published = run.advance_cycle(machine, input);
  require(published.bus.retirement && published.bus.progress &&
              machine.cpu().state().a == quarter && machine.cpu().state().x == reference::first &&
              run.elapsed_ticks() == 9 * period,
          "The two genuine results survive opcode replacement and elapsed DMA wait clocks");
  run.restore(saved, machine);
  const auto pending_write = run.advance_cycle(machine, input);
  require(pending_write.bus.dma.word && pending_write.bus.dma.core_stall &&
              machine.cpu().debug_state() == visible,
          "Joint recovery retains the actual pending DMA write and both instruction candidates");
  require(run.advance_cycle(machine, input).bus.retirement.has_value(),
          "The recovered older result publishes after the real pending DMA destination completes");
}

void held_predecessor_error_context() {
  DspMachine machine;
  prepare(machine);
  DspFetchRun run(contract(), machine);
  capture_both(run, machine);
  const auto visible = machine.cpu().debug_state();
  const auto saved = run.checkpoint(machine);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, clock::input(true))); });
  require(
      error.contains("X:0x00007e") && error.contains("DSP reference admission pc=0x000105") &&
          machine.cpu().debug_state() == visible && visible.cpu.pc == body && run.faulted(),
      "A younger real read failure reports its semantic PC while the older visible PC stays held");
  run.restore(saved, machine);
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  const auto recovered = run.advance_cycle(machine, both());
  require(recovered.bus.retirement && recovered.bus.progress && machine.cpu().state().a == quarter,
          "Joint recovery preserves both captured instructions after the contextual failure");
}

void partial_cycle_recovery(const bool unknown_fetch) {
  DspMachine machine;
  prepare(machine, false, unknown_fetch ? 2 : 16);
  if (unknown_fetch) {
    machine.bus().write(
        {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  }
  DspFetchRun run(contract(), machine);
  capture_both(run, machine);
  const auto before = machine.cpu().debug_state();
  const auto saved = run.checkpoint(machine);
  const auto error = fault([&] { static_cast<void>(run.advance_cycle(machine, both())); });
  require(run.faulted() && run.completed_cycles() == 5 && run.elapsed_ticks() == 5 * period &&
              machine.bus().last_dma_cycle() == 6 && error.contains("completed-cycles=5"),
          "A failed entered cycle quarantines the actual DMA/native prefix without advancing time");
  if (unknown_fetch) {
    require(error.contains("P:0x000108") && machine.cpu().debug_state() == before,
            "An earlier failed F2 leaves both native results unpublished");
  } else {
    require(error.contains("X:0x00007e") && machine.cpu().state().a == quarter &&
                machine.cpu().state().pc == body + 1 &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
            "A later failed real data read preserves already accepted predecessor retirement");
  }
  run.restore(saved, machine);
  if (unknown_fetch) { machine.bus().write({.address = clock::program + 8}); }
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  const auto recovered = run.advance_cycle(machine, both());
  require(recovered.bus.retirement && recovered.bus.progress && !run.faulted() &&
              machine.cpu().state().a == quarter,
          "Trusted joint recovery resumes both actual candidates from their saved admitted prefix");
}
}

void clock_checks() {
  shared_clock_publication();
  dma_holds_both();
  held_predecessor_error_context();
  partial_cycle_recovery(false);
  partial_cycle_recovery(true);
}
}
