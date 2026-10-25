// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_retirement_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <variant>

namespace blofeld::test::retirement {
namespace {
namespace clock = fetch_clock;
using native_access::fault;
constexpr std::uint64_t period = 45;
constexpr unsigned switched_omr = 0x600085;

DspFetchRunInputs contract() {
  auto inputs = clock::contract();
  inputs.execution = execution;
  return inputs;
}

void delayed_same_cursor_redirect() {
  DspMachine machine;
  constexpr unsigned branch_program_words = 18;
  std::array<std::uint32_t, branch_program_words> words{};
  words.front() = reference::long_jump;
  words.at(1) = clock::program + 2;
  clock::prepare(machine, words, {.execution = execution});
  auto invalid = contract();
  invalid.execution = DspReferenceExecution::access_boundaries;
  static_cast<void>(fault([&] { const DspFetchRun mismatch(invalid, machine); }));
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 4; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  const auto pending = machine.reference_pending();
  require(pending && std::holds_alternative<DspInstructionRetire>(pending->effect) &&
              std::get<DspInstructionRetire>(pending->effect).redirected &&
              std::get<DspInstructionRetire>(pending->effect).next_pc == clock::program + 2 &&
              machine.cpu().debug_state() == before && machine.reference_fetch().used == 1,
          "The actual JMP to its sequential cursor holds its redirect and old CPU, not a "
          "fabricated fallthrough result");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto held = run.advance_cycle(machine, clock::input());
    require(held.fetch.used == 2 && machine.cpu().debug_state() == before,
            "Independent automatic F2/F1 clocks continue while the real redirect result is held");
    const auto retired = run.advance_cycle(machine, clock::input(true));
    require(retired.bus.progress && retired.bus.progress->retired &&
                retired.bus.core_access_count == 1 && retired.fetch.used == 0 &&
                retired.issued_address == clock::program + 2 &&
                machine.cpu().state().pc == clock::program + 2 &&
                run.elapsed_ticks() == 6 * period &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
            "Actual retirement has no P footprint, flushes younger sampled words and issues the "
            "same-cursor target using the preserved native redirect outcome");
    static_cast<void>(run.begin_instruction(machine));
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
    const auto nop_result = run.advance_cycle(machine, clock::input(true));
    require(nop_result.bus.progress && !nop_result.bus.progress->retired &&
                nop_result.bus.progress->pending &&
                std::holds_alternative<DspInstructionRetire>(
                    nop_result.bus.progress->pending->effect) &&
                machine.cpu().state().pc == clock::program + 2,
            "The next begin retains the deferred contract and holds its actual queued zero NOP");
    const auto nop_retired = run.advance_cycle(machine, clock::input(true));
    require(nop_retired.bus.progress && nop_retired.bus.progress->retired &&
                machine.cpu().state().pc == clock::program + 3,
            "The zero NOP publishes only at its own later retirement boundary");
  }
}

void dma_cannot_replace_completed_result() {
  DspMachine machine;
  std::array<std::uint32_t, 16> words{};
  clock::prepare(machine,
                 words,
                 {.transfer = {.source = clock::program_source,
                               .destination = clock::program,
                               .control = clock::program_copy},
                  .execution = execution});
  const auto replacement = checkpoint::assemble("move #<$34,r0").front();
  machine.bus().write({.address = clock::program_source, .value = replacement});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  const auto pending = machine.reference_pending();
  require(machine.cpu().debug_state() == before && pending &&
              std::holds_alternative<DspInstructionRetire>(pending->effect),
          "The real zero NOP result is held before DMA changes its program word");
  auto input = clock::input();
  input.requests = 1;
  const auto source = run.advance_cycle(machine, input);
  require(source.bus.dma.phase && source.bus.dma.phase->kind == DspDmaPhaseKind::source_read,
          "The actual enabled DMA captures its independent P source while retirement is held");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    input.admit_native = true;
    input.grant = DspDmaGrant::wait;
    const auto wait = run.advance_cycle(machine, input);
    require(
        wait.bus.dma.core_stall && !wait.bus.progress && machine.cpu().debug_state() == before,
        "A conflicting held DMA destination also holds selected retirement with the F2 frontend");
    input.grant = DspDmaGrant::available;
    const auto write = run.advance_cycle(machine, input);
    require(write.bus.dma.core_stall && write.bus.dma.word &&
                write.bus.dma.word->value == replacement && !write.bus.progress &&
                machine.cpu().debug_state() == before,
            "The real replacement commits while the already completed NOP remains unpublished");
    input.requests = 0;
    const auto retired = run.advance_cycle(machine, input);
    require(
        retired.bus.progress && retired.bus.progress->retired &&
            retired.bus.core_access_count == 1 && machine.cpu().state().pc == clock::program + 1 &&
            machine.cpu().state().r[0] == clock::initial_r0 && run.elapsed_ticks() == 7 * period,
        "Later retirement publishes the original completed NOP, not the DMA replacement opcode");
    input.requests = 1;
  }
}

void memory_switch_starts_at_publication() {
  DspMachine machine;
  auto words = checkpoint::assemble("move #>$600085,omr");
  words.insert(words.end(), 16, 0);
  clock::prepare(machine, words, {.execution = execution});
  constexpr unsigned alias = 0x1000;
  constexpr unsigned source = 0x10000;
  machine.bus().write({.space = DspSpace::x, .address = source, .value = reference::first});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 4; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  const auto pending = machine.reference_pending();
  require(pending && std::get<DspInstructionRetire>(pending->effect).next_omr == switched_omr &&
              machine.cpu().debug_state() == before,
          "The real OMR result is retained without observing a memory switch prematurely");
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input()));
    static_cast<void>(fault([&] { static_cast<void>(machine.bus().inspect_program(alias)); }));
  }
  const auto retired = run.advance_cycle(machine, clock::input(true));
  require(retired.bus.progress && retired.bus.progress->retired &&
              machine.cpu().state().omr == switched_omr,
          "Publishing the native OMR begins its modeled switch delay");
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(fault([&] { static_cast<void>(machine.bus().inspect_program(alias)); }));
    static_cast<void>(run.advance_cycle(machine, clock::input()));
  }
  require(machine.bus().read({.address = alias}) == reference::first,
          "Three following completed clocks, excluding retirement itself, expose the real alias");
}

void failed_fetch_preserves_held_result() {
  DspMachine machine;
  const std::array words{0U, 0U, 0U};
  clock::prepare(machine, words, {.execution = execution});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, clock::input(true)));
  }
  static_cast<void>(run.advance_cycle(machine, clock::input()));
  const auto saved = run.checkpoint(machine);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, clock::input(true))); });
  require(error.contains("completed-cycles=4") && error.contains("DMA-index=5") &&
              error.contains("P:0x000103") && error.contains("pc=0x000100") && run.faulted() &&
              run.completed_cycles() == 4 && run.elapsed_ticks() == 4 * period &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 5,
          "An actual unknown F2 keeps its DMA prefix and prevents selected retirement or clock "
          "completion");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  run.restore(saved, machine);
  machine.bus().write({.address = clock::program + 3});
  machine.bus().write(
      {.address = clock::program, .value = checkpoint::assemble("move #<$34,r0").front()});
  const auto retired = run.advance_cycle(machine, clock::input(true));
  require(retired.bus.progress && retired.bus.progress->retired && !run.faulted() &&
              run.elapsed_ticks() == 5 * period &&
              machine.cpu().state().r[0] == clock::initial_r0 &&
              machine.cpu().state().pc == clock::program + 1,
          "Joint recovery retains the real pre-failure NOP candidate, even after its code changes");
}
}

void fetch_checks() {
  delayed_same_cursor_redirect();
  dma_cannot_replace_completed_result();
  memory_switch_starts_at_publication();
  failed_fetch_preserves_held_result();
}
}
