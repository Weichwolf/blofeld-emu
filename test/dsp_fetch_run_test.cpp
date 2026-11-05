// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_native_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::fetch_native;
namespace banks = blofeld::test::banks;
constexpr unsigned switched_omr = 0x600085;

OwnDspFetchCycleResult step(OwnDspMachine &machine, OwnDspTimeline &timeline) {
  const auto last = machine.bus().last_dma_cycle();
  const auto result = timeline.advance_one(timeline.next_deadline(),
                                           OwnDspBoundary::through,
                                           banks::inputs({.index = last ? *last + 1 : 0}));
  if (!result) { throw Fault("The actual next deadline must admit one reference cycle"); }
  healthy(*result);
  return *result;
}

void dma_stalls_and_clock() {
  const std::array<std::uint32_t, 16> words{};
  Fixture fixture(
      words,
      {.transfer = {.source = program_source, .destination = program, .control = program_copy},
       .timer = true});
  auto &machine = fixture.machine;
  machine.bus().write({.address = program_source, .value = short_move});
  fixture.attach();
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto first = fixture.advance(true, 1);
  require(
      !first.native && !first.captured && first.issued_address == program &&
          fixture.fetch->view().used == 0 && fixture.elapsed() == base_period &&
          machine.clock().timer_clk2_edges() == 0,
      "First clock issues actual F1 without admitting an unfetched instruction or a CLK/2 edge");
  const auto held = fixture.advance(true, 1, DspDmaGrant::wait);
  require(held.dma.core_stall && !held.native && !held.captured &&
              fixture.fetch->view().pending_address == program &&
              fixture.elapsed() == 2 * base_period && machine.clock().timer_clk2_edges() == 1 &&
              machine.bus().timers().read(count0) == load_value,
          "A stalled F2 retains its actual address while the real CLK/2 edge loads the timer");

  const auto waited = fixture.advance(true, 1, DspDmaGrant::wait);
  require(waited.dma.core_stall && !waited.captured && !waited.issued_address &&
              fixture.elapsed() == 3 * base_period,
          "Continued waiting retains the issued F1, held destination and elapsed wait period");
  const auto copied = fixture.advance(true, 1);
  require(copied.dma.word && copied.dma.word->value == short_move && copied.dma.core_stall &&
              !copied.captured && !copied.native && fixture.fetch->view().used == 0 &&
              fixture.elapsed() == 4 * base_period && machine.bus().timers().read(count0) == 8,
          "DMA destination commit still holds F2 while the second CLK/2 edge increments the timer");
  const auto fetched = fixture.advance(true);
  require(fetched.captured && !fetched.native && fetched.issued_address == program + 1 &&
              fixture.fetch->view().words.front().data == short_move &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress,
          "Actual later F2 samples the DMA-written opcode before any native result");
  const auto moved = fixture.advance(true);
  auto expected = before;
  expected.r[0] = copied_r0;
  require(moved.native && !moved.native->retired && machine.cpu().registers() == expected &&
              machine.cpu().progress() == progress && fixture.elapsed() == 6 * base_period &&
              machine.clock().timer_clk2_edges() == 3 &&
              machine.bus().timers().read(count0) == compare_value &&
              (machine.bus().timers().read(csr0, timer_flag) & timer_flag) != 0,
          "Native MOVE publication and independent timer compare precede CPU retirement");
  const auto pc = fixture.advance(true);
  ++expected.pc;
  require(pc.native && !pc.native->retired && machine.cpu().registers() == expected &&
              machine.cpu().progress() == progress,
          "Actual native PC publication is also separate from instruction retirement");
  const auto retired = fixture.advance(true);
  require(retired.native && retired.native->retired && machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 1 &&
              machine.cpu().progress().estimated_cycles == progress.estimated_cycles + 1 &&
              fixture.elapsed() == 8 * base_period && machine.clock().timer_clk2_edges() == 4 &&
              machine.bus().timers().read(count0) == compare_value + 1,
          "One native completion, eight elapsed periods and four CLK/2 edges remain distinct");
  for (unsigned i = 0; i < 2; ++i) { static_cast<void>(fixture.advance(true)); }
  ++expected.pc;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 2 &&
              fixture.elapsed() == 10 * base_period,
          "The next actual queued NOP publishes PC and retires without another opcode admission "
          "read");
}

void automatic_fetch_and_backpressure() {
  const std::array<std::uint32_t, 16> words{};
  Fixture fixture(words);
  fixture.attach();
  const auto before = fixture.machine.cpu().registers();
  const auto progress = fixture.machine.cpu().progress();
  for (unsigned cycle = 1; cycle <= 10; ++cycle) {
    const auto result = fixture.advance();
    const auto expected = cycle < 9 ? cycle - 1 : 8;
    require(fixture.fetch->view().used == expected && fixture.elapsed() == cycle * base_period &&
                fixture.machine.cpu().registers() == before &&
                fixture.machine.cpu().progress() == progress,
            "Automatic F1/F2 fills one real word per period while the selected CPU is held");
    if (cycle < 9) {
      require(result.issued_address == program + cycle - 1,
              "Every new F1 comes from the actual sequential cursor");
    } else {
      require(!result.issued_address && !fixture.fetch->view().pending_address,
              "Finite host continuation capacity applies real backpressure");
    }
  }
  require(fixture.machine.clock().timer_clk2_edges() == 5,
          "CLK/2 counts held and backpressure periods");
  const auto published = fixture.advance(true);
  auto expected = before;
  ++expected.pc;
  require(published.native && !published.native->retired &&
              published.issued_address == program + 8 && fixture.fetch->view().used == 7 &&
              fixture.machine.cpu().registers() == expected &&
              fixture.machine.cpu().progress() == progress,
          "One native dequeue and actual PC publication release capacity without a fabricated "
          "retirement");
  const auto retired = fixture.advance(true);
  require(retired.native && retired.native->retired && !retired.issued_address &&
              fixture.fetch->view().used == 8 && fixture.machine.cpu().registers() == expected &&
              fixture.machine.cpu().progress().instructions == progress.instructions + 1,
          "The following real F2 refills capacity while the prior actual NOP retires");
}

void pll_and_odd_phase() {
  const std::array<std::uint32_t, 16> words{};
  Fixture fixture(words, {.timer = true});
  fixture.attach();
  static_cast<void>(fixture.advance());

  fixture.machine.bus().pll().write(faster);
  for (unsigned cycle = 2; cycle <= 4; ++cycle) {
    const auto before = fixture.machine.clock().elapsed_ticks();
    static_cast<void>(fixture.advance());
    require(fixture.machine.clock().elapsed_ticks() == before + faster_period &&
                fixture.elapsed() == base_period + (cycle - 1) * faster_period,
            "Actual PCTL changes only subsequent reference periods without retiming the prefix");
  }
  require(fixture.machine.clock().native_cycle_units() == 4 &&
              fixture.machine.clock().timer_clk2_edges() == 2 &&
              fixture.machine.bus().timers().read(count0) == 8,
          "Clock progress preserves odd CLK/2 residue, timer state and the PLL command");
}

void memory_switch_delay() {
  const std::array<std::uint32_t, 16> words{long_omr, switched_omr};
  Fixture fixture(words);
  fixture.attach();
  constexpr unsigned alias = 0x1000;
  constexpr unsigned source = 0x10000;
  constexpr unsigned datum = 0x123456;
  fixture.machine.bus().write({.space = DspSpace::x, .address = source, .value = datum});
  const auto before = fixture.machine.cpu().registers();
  const auto progress = fixture.machine.cpu().progress();
  for (unsigned cycle = 0; cycle < 4; ++cycle) { static_cast<void>(fixture.advance(true)); }
  require(fixture.machine.cpu().registers().omr == switched_omr &&
              fixture.machine.cpu().progress() == progress,
          "Actual native OMR publication requests memory switching before later retirement");
  for (unsigned delay = 0; delay < 3; ++delay) {
    static_cast<void>(
        rejects([&] { static_cast<void>(fixture.machine.bus().inspect_program(alias)); }));
    static_cast<void>(fixture.advance(delay < 2));
  }
  auto expected = before;
  expected.omr = switched_omr;
  expected.pc = program + 2;
  require(fixture.machine.cpu().registers() == expected &&
              fixture.machine.cpu().progress().instructions == progress.instructions + 1 &&
              fixture.machine.bus().read({.address = alias}) == datum &&
              fixture.elapsed() == 7 * base_period,
          "Memory switch takes three subsequent completed periods, excluding its actual OMR "
          "command cycle");
}

void captured_opcode_and_failure() {
  const std::array words{0U};
  for (const bool rewrite : {false, true}) {
    Fixture fixture(words);
    OwnDspTimeline timeline(fixture.machine, timing, 0);
    const auto before = fixture.machine.cpu().registers();
    const auto progress = fixture.machine.cpu().progress();
    for (unsigned i = 0; i < 2; ++i) { static_cast<void>(step(fixture.machine, timeline)); }
    fixture.machine.bus().write({.address = program, .value = short_move});
    static_cast<void>(step(fixture.machine, timeline));
    const auto retired = step(fixture.machine, timeline);
    auto expected = before;
    ++expected.pc;
    require(retired.native && retired.native->retired &&
                fixture.machine.cpu().registers() == expected &&
                fixture.machine.cpu().progress().instructions == progress.instructions + 1,
            "The captured original NOP retires despite replacement of its backing opcode");
    if (rewrite) { fixture.machine.bus().write({.address = program + 1}); }
    const auto error = rejects([&] { static_cast<void>(step(fixture.machine, timeline)); });
    require(error.contains("Unknown DSP program sample read P:0x000101") &&
                error.contains("pc=0x000101") && error.contains("omr=0x000305") &&
                timeline.faulted() && fixture.elapsed() == 4 * base_period &&
                fixture.machine.bus().last_dma_cycle() == 4 &&
                fixture.machine.cpu().registers() == expected,
            "Captured unknown opcodes quarantine before DMA, including after later RAM writes");
    static_cast<void>(rejects([&] { static_cast<void>(step(fixture.machine, timeline)); }));
    require(fixture.elapsed() == 4 * base_period && fixture.machine.bus().last_dma_cycle() == 4 &&
                fixture.machine.cpu().registers() == expected &&
                fixture.machine.cpu().progress().instructions == progress.instructions + 1,
            "A failed timeline cannot repeat effects or advance native clocks");
  }
}

void unknown_extension_admission() {
  constexpr unsigned immediate = 0x123456;
  const std::array words{long_r0};
  for (const bool rewrite : {false, true}) {
    Fixture fixture(words);
    OwnDspTimeline timeline(fixture.machine, timing, 0);
    const auto before = fixture.machine.cpu().registers();
    const auto progress = fixture.machine.cpu().progress();
    for (unsigned i = 0; i < 3; ++i) { static_cast<void>(step(fixture.machine, timeline)); }
    require(fixture.machine.cpu().registers() == before &&
                fixture.machine.cpu().progress() == progress &&
                !fixture.machine.cpu().pending_instruction(),
            "An absent extension F2 cannot authorize an incomplete native instruction");
    if (rewrite) { fixture.machine.bus().write({.address = program + 1, .value = immediate}); }
    const auto error = rejects([&] { static_cast<void>(step(fixture.machine, timeline)); });
    require(error.contains("Unknown DSP program sample read P:0x000101") &&
                error.contains("pc=0x000100") && timeline.faulted() &&
                fixture.elapsed() == 3 * base_period &&
                fixture.machine.bus().last_dma_cycle() == 3 &&
                fixture.machine.cpu().registers() == before &&
                fixture.machine.cpu().progress() == progress,
            "Captured unknown extensions fail before CPU work or DMA despite later RAM writes");
  }
  Fixture fixture(words);
  OwnDspTimeline timeline(fixture.machine, timing, 0);
  const auto before = fixture.machine.cpu().registers();
  const auto progress = fixture.machine.cpu().progress();
  for (unsigned i = 0; i < 2; ++i) { static_cast<void>(step(fixture.machine, timeline)); }
  fixture.machine.bus().write({.address = program + 1, .value = immediate});
  fixture.machine.bus().write({.address = program, .value = short_move});
  for (unsigned i = 0; i < 4; ++i) { static_cast<void>(step(fixture.machine, timeline)); }
  auto expected = before;
  expected.pc = program + 2;
  expected.r[0] = immediate;
  require(fixture.machine.cpu().registers() == expected && fixture.elapsed() == 6 * base_period &&
              fixture.machine.cpu().progress() ==
                  dsp56300::Progress{.instructions = progress.instructions + 1,
                                     .estimated_cycles = progress.estimated_cycles + 2},
          "Initialization before extension F2 supplies the captured long MOVE and its base cost");
}

}

int main() {
  try {
    dma_stalls_and_clock();
    automatic_fetch_and_backpressure();
    pll_and_odd_phase();
    memory_switch_delay();
    captured_opcode_and_failure();
    unknown_extension_admission();
    guard_checks();
    std::cout << "Native clocked F1/F2, timer/PLL, pipeline ownership and failure checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
