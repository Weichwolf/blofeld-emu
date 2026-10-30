// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_writeback_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <cstdint>
#include <variant>

namespace blofeld::test::writeback {
namespace {
namespace clock = fetch_clock;
using native_access::fault;
constexpr std::uint64_t period = 45;

DspFetchRunInputs contract() {
  auto result = clock::contract();
  result.execution = execution;
  return result;
}

auto words() {
  auto result = checkpoint::assemble("move x:(r0)+,x0");
  result.insert(result.end(), 16, 0);
  return result;
}

void separate_e1_e2_clocks() {
  DspMachine machine;
  clock::prepare(machine, words(), {.execution = execution});
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  const auto before = machine.cpu().debug_state();
  auto visible = before;
  visible.cpu.r[0] = clock::initial_r0 + 1;
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 5; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  require(machine.cpu().debug_state() == visible && run.completed_cycles() == 5,
          "Actual F1/F2/address/data clocks precede the held E1 register result");
  const auto before_e1 = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(before_e1, machine);
    machine.bus().write(
        {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::third});
    const auto e1 = run.advance_cycle(machine, clock::input(true));
    visible.cpu.x = reference::first;
    require(e1.bus.progress && e1.bus.progress->completed &&
                std::holds_alternative<DspMoveWriteback>(*e1.bus.progress->completed) &&
                e1.bus.core_access_count == 1 && machine.cpu().debug_state() == visible &&
                run.elapsed_ticks() == 6 * period,
            "E1/F2 share a reference clock with no extra data read, PC advance or SDK retirement");
    const auto after_e1 = run.checkpoint(machine);
    const auto e2 = run.advance_cycle(machine, clock::input(true));
    require(e2.bus.progress && e2.bus.progress->retired &&
                machine.cpu().state().pc == clock::program + 1 &&
                machine.cpu().state().x == reference::first && run.elapsed_ticks() == 7 * period,
            "The following selected E2 retires the held native result once");
    run.restore(after_e1, machine);
    const auto repeated = run.advance_cycle(machine, clock::input(true));
    require(repeated.bus.progress && repeated.bus.progress->retired &&
                machine.cpu().state().x == reference::first,
            "A post-E1 joint checkpoint restores actual published data and pending E2 together");
  }
}

void dma_wait_retains_e1() {
  DspMachine machine;
  clock::prepare(machine,
                 words(),
                 {.transfer = {.source = clock::program_source,
                               .destination = clock::program,
                               .control = clock::program_copy},
                  .execution = execution});
  machine.bus().write({.address = clock::program_source});
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  const auto before = machine.cpu().debug_state();
  auto visible = before;
  visible.cpu.r[0] = clock::initial_r0 + 1;
  visible.cpu.x = reference::first;
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 5; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  auto input = clock::input(true);
  input.requests = 1;
  const auto e1 = run.advance_cycle(machine, input);
  require(e1.bus.dma.phase && e1.bus.dma.phase->kind == DspDmaPhaseKind::source_read &&
              machine.cpu().debug_state() == visible,
          "A real DMA source overlaps E1 while the completed CPU result remains pending for E2");
  const auto saved = run.checkpoint(machine);
  input.grant = DspDmaGrant::wait;
  const auto held = run.advance_cycle(machine, input);
  require(held.bus.dma.core_stall && !held.bus.progress && machine.cpu().debug_state() == visible,
          "Held DMA/F2 cannot undo already published E1 or prematurely retire E2");
  input.grant = DspDmaGrant::available;
  const auto copied = run.advance_cycle(machine, input);
  require(copied.bus.dma.word && copied.bus.dma.core_stall &&
              machine.cpu().debug_state() == visible,
          "Actual opcode replacement commits while E2 remains held");
  input.requests = 0;
  const auto retired = run.advance_cycle(machine, input);
  require(retired.bus.progress && retired.bus.progress->retired &&
              machine.cpu().state().x == reference::first && run.elapsed_ticks() == 9 * period,
          "E2 retires the original instruction and its accepted E1 data after real DMA waits");
  run.restore(saved, machine);
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::third});
  input.requests = 0;
  const auto replayed_dma = run.advance_cycle(machine, input);
  require(replayed_dma.bus.dma.word && replayed_dma.bus.dma.core_stall &&
              machine.cpu().debug_state() == visible,
          "Restoring after E1 retains the pending DMA destination even with no new request");
  const auto recovered = run.advance_cycle(machine, input);
  require(recovered.bus.progress && recovered.bus.progress->retired &&
              machine.cpu().state().x == reference::first,
          "Joint recovery after E1 preserves the real captured load despite later memory changes");
}

void mapped_unknown_f2_preserves_accepted_e1() {
  DspMachine machine;
  auto program = checkpoint::assemble("move x:(r0)+,x0");
  program.insert(program.end(), 4, 0);
  clock::prepare(machine, program, {.execution = execution});
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  const auto before = machine.cpu().debug_state();
  auto visible = before;
  visible.cpu.r[0] = clock::initial_r0 + 1;
  visible.cpu.x = reference::first;
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 6; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  require(machine.cpu().debug_state() == visible, "The real E1 result has actually been accepted");
  const auto saved = run.checkpoint(machine);
  const auto retired = run.advance_cycle(machine, clock::input(true));
  auto expected = visible.cpu;
  expected.pc = clock::program + 1;
  require(retired.bus.progress && retired.bus.progress->retired && !run.faulted() &&
              run.completed_cycles() == 7 && run.elapsed_ticks() == 7 * period &&
              retired.fetch.used == 5 && retired.fetch.words[4].address == clock::program + 5 &&
              !retired.fetch.words[4].data && machine.cpu().state() == expected,
          "Mapped unknown speculative F2 remains absent;it cannot undo accepted E1 or block E2");
  run.restore(saved, machine);
  machine.bus().write({.address = clock::program + 5});
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::third});
  const auto recovered = run.advance_cycle(machine, clock::input(true));
  require(
      recovered.bus.progress && recovered.bus.progress->retired && !run.faulted() &&
          machine.cpu().state().x == reference::first &&
          machine.cpu().state().pc == clock::program + 1,
      "Trusted joint restore retains accepted E1 and original pending E2 despite a new F2 value");
}

void physical_f2_preserves_accepted_e1() {
  constexpr unsigned edge = 0xffb;
  constexpr unsigned unavailable = 0x1000;
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", clock::initial_r0);
  for (const auto address :
       {dma_test::source0, dma_test::destination0, dma_test::counter0, dma_test::control0}) {
    code.emit("movep #>$0,x:<<" + number(address));
  }
  code.word(reference::long_jump);
  code.word(edge);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  auto program = checkpoint::assemble("move x:(r0)+,x0");
  program.insert(program.end(), 4, 0);
  checkpoint::ram(machine.bus(), edge, program);
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  checkpoint::run(machine, 6);
  require(machine.cpu().state().pc == edge,
          "Own native jump enters the actual last five mapped program words");
  static_cast<void>(machine.begin_reference_instruction(execution));
  machine.bus().write(
      {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::first});
  auto visible = machine.cpu().debug_state();
  visible.cpu.r[0] = clock::initial_r0 + 1;
  visible.cpu.x = reference::first;
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 6; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  require(machine.cpu().debug_state() == visible,
          "Boundary case accepts the real load E1 before physical F2 fails");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    if (replay != 0) { run.restore(saved, machine); }
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, clock::input(true))); });
    require(error.contains("P:" + hex(unavailable, 6)) && error.contains("completed-cycles=6") &&
                error.contains("pc=" + hex(edge, 6)) && run.faulted() &&
                run.elapsed_ticks() == 6 * period && machine.bus().last_dma_cycle() == 6 &&
                machine.cpu().debug_state() == visible,
            "Physical F2 admission preserves E1 before DMA/E2/time: " + error);
  }
}
}

void clock_checks() {
  separate_e1_e2_clocks();
  dma_wait_retains_e1();
  mapped_unknown_f2_preserves_accepted_e1();
  physical_f2_preserves_accepted_e1();
}
}
