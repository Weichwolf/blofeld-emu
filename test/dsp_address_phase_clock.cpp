// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_address_phase_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <cstdint>
#include <variant>

namespace blofeld::test::address_phase {
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
  require(result.size() == 1, "The own direct postincrement move is one program word");
  result.insert(result.end(), 16, 0);
  return result;
}

void unknown_read_after_published_address() {
  DspMachine machine;
  clock::prepare(machine, words(), {.execution = execution});
  const auto before = machine.cpu().debug_state();
  auto visible = before;
  visible.cpu.r[0] = clock::initial_r0 + 1;
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  const auto pending = machine.reference_pending();
  require(machine.cpu().debug_state() == before && pending &&
              std::holds_alternative<DspAddressUpdate>(pending->effect),
          "Native decode exposes a genuine pre-read address result without reading unknown RAM");
  const auto address = run.advance_cycle(machine, clock::input(true));
  require(address.bus.progress && address.bus.progress->completed &&
              std::holds_alternative<DspAddressUpdate>(*address.bus.progress->completed) &&
              address.bus.core_access_count == 1 && machine.cpu().debug_state() == visible &&
              run.completed_cycles() == 4 && run.elapsed_ticks() == 4 * period,
          "The selected address update coexists with F2 and advances one real reference clock");
  const auto saved = run.checkpoint(machine);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, clock::input(true))); });
  require(error.contains("Unknown DSP memory read at X:0x00007e") &&
              error.contains("completed-cycles=4") && error.contains("pc=0x000100") &&
              run.faulted() && run.elapsed_ticks() == 4 * period &&
              machine.cpu().debug_state() == visible && machine.bus().last_dma_cycle() == 5,
          "The later actual unknown load preserves the earlier R publication and DMA prefix");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    machine.bus().write(
        {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::second});
    const auto sampled = run.advance_cycle(machine, clock::input(true));
    require(sampled.bus.progress && !sampled.bus.progress->retired &&
                sampled.bus.core_access_count == 2 && machine.cpu().debug_state() == visible,
            "Joint restore preserves the old effective address while a real X read overlaps F2");
    machine.bus().write(
        {.space = DspSpace::x, .address = clock::initial_r0, .value = reference::third});
    const auto retired = run.advance_cycle(machine, clock::input(true));
    require(
        retired.bus.progress && retired.bus.progress->retired &&
            machine.cpu().state().x == reference::second &&
            machine.cpu().state().r[0] == clock::initial_r0 + 1 &&
            machine.cpu().state().pc == clock::program + 1 && run.elapsed_ticks() == 6 * period,
        "Retirement retains the genuine sampled word and single AGU result through RAM overwrite");
  }
}

void dma_wait_retains_published_address() {
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
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  auto input = clock::input(true);
  input.requests = 1;
  const auto admitted = run.advance_cycle(machine, input);
  require(admitted.bus.dma.phase && admitted.bus.dma.phase->kind == DspDmaPhaseKind::source_read &&
              machine.cpu().debug_state() == visible,
          "Actual DMA source and address publication proceed without any native data read");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    input.grant = DspDmaGrant::wait;
    const auto held = run.advance_cycle(machine, input);
    require(
        held.bus.dma.core_stall && !held.bus.progress && machine.cpu().debug_state() == visible,
        "A held destination/F2 conflict cannot undo or repeat an already admitted address phase");
    input.grant = DspDmaGrant::available;
    const auto copied = run.advance_cycle(machine, input);
    require(copied.bus.dma.word && copied.bus.dma.word->value == 0 && copied.bus.dma.core_stall &&
                !copied.bus.progress && machine.cpu().debug_state() == visible,
            "The real DMA opcode overwrite completes while the selected X load remains held");
    input.requests = 0;
    const auto sampled = run.advance_cycle(machine, input);
    require(sampled.bus.progress && !sampled.bus.progress->retired &&
                machine.cpu().debug_state() == visible,
            "The resumed real read uses the original instruction and pre-update effective address");
    const auto retired = run.advance_cycle(machine, input);
    require(
        retired.bus.progress && retired.bus.progress->retired &&
            machine.cpu().state().x == reference::first &&
            machine.cpu().state().r[0] == clock::initial_r0 + 1 && run.completed_cycles() == 8 &&
            run.elapsed_ticks() == 8 * period,
        "Native retirement preserves the real load and one address update across actual DMA waits");
    input.requests = 1;
  }
}
}

void clock_checks() {
  unknown_read_after_published_address();
  dma_wait_retains_published_address();
}
}
