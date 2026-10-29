// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::fetch_clock;
namespace access = blofeld::test::reference;
using native_access::fault;
constexpr std::uint64_t base_period_ticks = 45;
constexpr std::uint64_t faster_period_ticks = 10;
constexpr unsigned switched_omr = 0x600085;

bool retired(const DspFetchCycleResult &result) {
  return result.bus.progress && result.bus.progress->retired;
}

void dma_stalls_clock_and_replay() {
  DspMachine machine;
  std::array<std::uint32_t, 16> words{};
  prepare(machine,
          words,
          {.transfer = {.source = program_source, .destination = program, .control = program_copy},
           .timer = true});
  const auto opcode = checkpoint::assemble("move #<$34,r0").front();
  machine.bus().write({.address = program_source, .value = opcode});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  auto request = input(true);
  request.requests = 1;
  const auto first = run.advance_cycle(machine, request);
  require(first.native_starved && !first.bus.progress && first.fetch.used == 0 &&
              first.issued_address == program && first.period_ticks == base_period_ticks &&
              first.completion_ticks == base_period_ticks && run.timer_clk2_edges() == 0,
          "First clock issues real F1; an unfetched instruction cannot admit in the same cycle");
  request.grant = DspDmaGrant::wait;
  const auto held = run.advance_cycle(machine, request);
  require(held.bus.dma.core_stall && held.fetch.used == 0 &&
              held.fetch.pending_address == program && run.completed_cycles() == 2 &&
              run.elapsed_ticks() == 2 * base_period_ticks && run.timer_clk2_edges() == 1 &&
              machine.bus().timers().read(count0) == load_value,
          "The stalled F2 keeps its actual address while a completed core clock advances CLK/2");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    request.grant = DspDmaGrant::wait;
    const auto waited = run.advance_cycle(machine, request);
    require(waited.bus.dma.core_stall && !waited.issued_address &&
                run.elapsed_ticks() == 3 * base_period_ticks,
            "A restored held destination retains the old F1 address and real elapsed wait clock");
    request.grant = DspDmaGrant::available;
    const auto copied = run.advance_cycle(machine, request);
    require(copied.bus.dma.word && copied.bus.dma.word->value == opcode &&
                copied.bus.dma.core_stall && !copied.bus.progress && copied.fetch.used == 0 &&
                run.elapsed_ticks() == 4 * base_period_ticks &&
                machine.bus().timers().read(count0) == 8,
            "Actual DMA completion still holds F2 while the second CLK/2 edge advances the timer");
    request.requests = 0;
    const auto fetched = run.advance_cycle(machine, request);
    require(fetched.native_starved && !fetched.bus.progress &&
                fetched.fetch.words.front().data == opcode &&
                fetched.issued_address == program + 1 && machine.cpu().debug_state() == before,
            "The following F2 samples DMA's word and overlaps the next F1 without early native "
            "retirement");
    const auto executed = run.advance_cycle(machine, request);
    require(retired(executed) && executed.issued_address == program + 2 &&
                run.completed_cycles() == 6 && run.elapsed_ticks() == 6 * base_period_ticks &&
                run.timer_clk2_edges() == 3 && machine.cpu().state().r[0] == copied_r0 &&
                machine.cpu().state().pc == program + 1 &&
                machine.cpu().debug_state().native_cycles == before.native_cycles + 1 &&
                machine.bus().timers().read(count0) == compare_value &&
                (machine.bus().timers().read(csr0, timer_flag) & timer_flag) != 0,
            "Elapsed waits, real instruction result, independent timer compare and SDK units "
            "remain distinct");
    static_cast<void>(run.begin_instruction(machine));
    const auto nop = run.advance_cycle(machine, input(true));
    require(retired(nop) && nop.issued_address == program + 3 &&
                run.elapsed_ticks() == 7 * base_period_ticks &&
                machine.cpu().state().pc == program + 2 && machine.cpu().state().r[0] == copied_r0,
            "The following queued zero word executes without a second program read");
    request.requests = 1;
  }
}

void automatic_fetch_and_backpressure() {
  DspMachine machine;
  std::array<std::uint32_t, 16> words{};
  prepare(machine, words);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 1; cycle <= 10; ++cycle) {
    const auto result = run.advance_cycle(machine, input());
    const auto expected = cycle < 9 ? cycle - 1 : 8;
    require(result.completed_cycles == cycle && result.fetch.used == expected &&
                result.completion_ticks == static_cast<std::uint64_t>(cycle) * base_period_ticks &&
                machine.cpu().debug_state() == before,
            "Primary F1/F2 word latency and one sampled word per clock are independent of native "
            "execution");
    if (cycle < 9) {
      require(result.issued_address == program + cycle - 1,
              "Every F1 address is derived sequentially without caller supplied fetch commands");
    } else {
      require(!result.issued_address && !result.fetch.pending_address,
              "Host continuation capacity applies backpressure without a fictitious memory read");
    }
  }
  require(run.timer_clk2_edges() == 5, "Clock/2 retains completed idle and backpressure cycles");
  const auto released = run.advance_cycle(machine, input(true));
  require(retired(released) && released.bus.core_access_count == 0 &&
              released.issued_address == program + 8 && released.fetch.used == 7 &&
              machine.cpu().state().pc == program + 1,
          "A genuine native dequeue releases capacity for the next automatic F1");
  static_cast<void>(run.begin_instruction(machine));
  const auto steady = run.advance_cycle(machine, input(true));
  require(retired(steady) && steady.issued_address == program + 9 && steady.fetch.used == 7 &&
              machine.cpu().state().pc == program + 2,
          "Old F2, queued native admission and new F1 coexist under the explicit reference order");
}

void pll_and_odd_phase_restore() {
  DspMachine machine;
  std::array<std::uint32_t, 16> words{};
  prepare(machine, words, {.timer = true});
  DspFetchRun run(contract(), machine);
  static_cast<void>(run.advance_cycle(machine, input()));
  const auto saved = run.checkpoint(machine);
  constexpr unsigned faster = 0x02e112;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    machine.bus().pll().write(faster);
    for (unsigned cycle = 2; cycle <= 4; ++cycle) {
      const auto result = run.advance_cycle(machine, input());
      require(result.period_ticks == faster_period_ticks &&
                  result.completion_ticks == base_period_ticks + (cycle - 1) * faster_period_ticks,
              "A new declared PCTL target changes the next reference clock period, not prior "
              "elapsed time");
    }
    require(run.completed_cycles() == 4 &&
                run.elapsed_ticks() == base_period_ticks + 3 * faster_period_ticks &&
                run.timer_clk2_edges() == 2 && machine.bus().timers().read(count0) == 8,
            "Joint restore retains odd CLK/2 residue, actual timer and initial PLL command state");
  }
}

void memory_switch_delay() {
  DspMachine machine;
  auto words = checkpoint::assemble("move #>$600085,omr");
  require(words.size() == 2, "Independent long-immediate OMR instruction length");
  words.insert(words.end(), 14, 0);
  prepare(machine, words);
  constexpr unsigned alias = 0x1000;
  constexpr unsigned source = 0x10000;
  machine.bus().write({.space = DspSpace::x, .address = source, .value = access::first});
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 4; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, input(true)));
  }
  require(machine.cpu().state().omr == switched_omr && machine.cpu().state().pc == program + 2,
          "The selected real native OMR command requests memory switching");
  for (unsigned delay = 0; delay < 3; ++delay) {
    static_cast<void>(fault([&] { static_cast<void>(machine.bus().inspect_program(alias)); }));
    static_cast<void>(run.advance_cycle(machine, input()));
  }
  require(
      run.completed_cycles() == 7 && machine.bus().read({.address = alias}) == access::first,
      "The switch takes three subsequent completed core clocks, excluding its own command cycle");
}

void failure_and_joint_recovery() {
  DspMachine machine;
  const std::array words{0U};
  prepare(machine, words);
  DspFetchRun run(contract(), machine);
  static_cast<void>(run.advance_cycle(machine, input()));
  static_cast<void>(run.advance_cycle(machine, input()));
  const auto saved = run.checkpoint(machine);
  const auto before = machine.cpu().debug_state();
  const auto sampled = run.advance_cycle(machine, input(true));
  auto retired_before = before;
  retired_before.cpu.pc = program + 1;
  ++retired_before.native_instructions;
  ++retired_before.native_cycles;
  require(retired(sampled) && sampled.fetch.used == 1 &&
              sampled.fetch.words.front().address == program + 1 &&
              !sampled.fetch.words.front().data && machine.cpu().debug_state() == retired_before,
          "Mapped unknown F2 preserves its absent value while the older genuine NOP retires");
  const auto absent = run.checkpoint(machine);
  for (const bool rewrite : {false, true}) {
    run.restore(absent, machine);
    if (rewrite) { machine.bus().write({.address = program + 1}); }
    static_cast<void>(run.begin_instruction(machine));
    const auto error = fault([&] { static_cast<void>(run.advance_cycle(machine, input(true))); });
    require(error.contains("completed-cycles=3") && error.contains("DMA-index=4") &&
                error.contains("Unknown DSP program sample read P:0x000101") &&
                error.contains("pc=0x000101") && error.contains("omr=0x000305") && run.faulted() &&
                run.completed_cycles() == 3 && run.elapsed_ticks() == 3 * base_period_ticks &&
                machine.bus().last_dma_cycle() == 3 &&
                machine.cpu().debug_state() == retired_before,
            "Actual unknown opcode admission refuses before DMA; restore or later RAM writes "
            "cannot repair the captured absent value");
    static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
    static_cast<void>(fault([&] { static_cast<void>(run.checkpoint(machine)); }));
  }
  run.restore(saved, machine);
  machine.bus().write({.address = program + 1});
  machine.bus().write({.address = program, .value = checkpoint::assemble("move #<$34,r0").front()});
  const auto recovered = run.advance_cycle(machine, input(true));
  require(
      !run.faulted() && retired(recovered) && run.completed_cycles() == 3 &&
          run.elapsed_ticks() == 3 * base_period_ticks && machine.cpu().state().r[0] == initial_r0,
      "Joint restore recovers clock/index/queue; an already sampled zero survives RAM overwrite");
}

void unknown_extension_admission() {
  constexpr unsigned immediate_value = 0x123456;
  DspMachine machine;
  const auto words = checkpoint::assemble("move #>$123456,r0");
  require(words.size() == 2, "Own long immediate has an independently expected extension word");
  prepare(machine, std::span(words).first(1));
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  static_cast<void>(run.advance_cycle(machine, input()));
  static_cast<void>(run.advance_cycle(machine, input()));
  const auto before_extension = run.checkpoint(machine);
  const auto opcode = run.advance_cycle(machine, input(true));
  const auto pending = machine.reference_pending();
  require(!retired(opcode) && pending &&
              std::get<DspNativeAccess>(pending->effect).kind == DspNativeAccessKind::extension &&
              opcode.fetch.used == 1 && opcode.fetch.words.front().address == program + 1 &&
              !opcode.fetch.words.front().data && machine.cpu().debug_state() == before,
          "Actual opcode admission and absent extension F2 complete without instruction effects");
  const auto absent = run.checkpoint(machine);
  for (const bool rewrite : {false, true}) {
    run.restore(absent, machine);
    if (rewrite) { machine.bus().write({.address = program + 1, .value = words.at(1)}); }
    const auto error = fault([&] { static_cast<void>(run.advance_cycle(machine, input(true))); });
    require(error.contains("Unknown DSP program sample read P:0x000101") &&
                error.contains("pc=0x000100") && error.contains("omr=0x000305") && run.faulted() &&
                run.completed_cycles() == 3 && run.elapsed_ticks() == 3 * base_period_ticks &&
                machine.bus().last_dma_cycle() == 3 && machine.cpu().debug_state() == before,
            "Actual absent extension refuses before DMA or CPU effects despite a later RAM write");
  }
  run.restore(before_extension, machine);
  machine.bus().write({.address = program + 1, .value = words.at(1)});
  static_cast<void>(run.advance_cycle(machine, input(true)));
  const auto recovered = run.advance_cycle(machine, input(true));
  auto expected = before;
  expected.cpu.pc = program + 2;
  expected.cpu.r[0] = immediate_value;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(retired(recovered) && run.completed_cycles() == 4 &&
              run.elapsed_ticks() == 4 * base_period_ticks &&
              machine.cpu().debug_state() == expected,
          "A genuinely fresh extension F2 executes the own long immediate with full native state");
}

void uncoordinated_restore_guard() {
  DspMachine machine;
  std::array<std::uint32_t, 16> words{};
  prepare(machine, words);
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    static_cast<void>(run.advance_cycle(machine, input(true)));
  }
  require(!machine.reference_pending(),
          "Own queued NOP has actually retired before the ownership check");
  const auto machine_only = machine.checkpoint();
  const auto joint = run.checkpoint(machine);
  const auto before = machine.cpu().debug_state();
  machine.restore(machine_only);
  require(machine.cpu().debug_state() == before,
          "All exposed native registers/counters are identical after restore");
  require(fault([&] {
            static_cast<void>(run.advance_cycle(machine, input()));
          }).contains("coordinated native/frontend/DMA"),
          "An identical machine-only restore still invalidates the process-local clock owner "
          "capability");
  require(run.completed_cycles() == 3 && machine.bus().last_dma_cycle() == 3,
          "Rejected uncoordinated restore performs no DMA sampling or clock advance");
  run.restore(joint, machine);
  const auto continued = run.advance_cycle(machine, input());
  require(continued.completed_cycles == 4 && run.elapsed_ticks() == 4 * base_period_ticks,
          "A trusted joint restore refreshes the capability while retaining exact elapsed clocks");
}
}

int main() {
  try {
    dma_stalls_clock_and_replay();
    automatic_fetch_and_backpressure();
    pll_and_odd_phase_restore();
    memory_switch_delay();
    failure_and_joint_recovery();
    unknown_extension_admission();
    uncoordinated_restore_guard();
    guard_checks();
    std::cout << "Clocked automatic DSP F1/F2 checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
