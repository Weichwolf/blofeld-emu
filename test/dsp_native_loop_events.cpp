// SPDX-License-Identifier: GPL-3.0-only
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp_fetch_native_fixture.hpp"
#include "dsp_native_loop_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace blofeld::test::native_loop {
namespace {
using fixture::require;
namespace native = dma_test;
constexpr unsigned count = 3, width = 4, total_periods = 17, instructions = 13;
constexpr unsigned payload = 0x123456, sentinel = 0x765432;
constexpr unsigned program_destination = 0x300, copy_x_to_p = 0x880248;

void finish(fixture::Fixture &f, const std::uint64_t target) {
  constexpr unsigned bound = 128;
  for (unsigned index = 0; f.machine.cpu().progress().instructions < target && index < bound;
       ++index) {
    static_cast<void>(advance(f));
  }
  require(f.machine.cpu().progress().instructions == target,
          "The connected authored event/loop suffix reaches its literal instruction count");
}

void dma(const bool conflict) {
  std::array<std::uint32_t, padding_words> code{};
  code[0] = dor_three;
  code[1] = 5;
  const auto destination = conflict ? program_destination : native::destination;
  const auto space = conflict ? DspSpace::p : DspSpace::y;
  fixture::Fixture f(code,
                     {.transfer = {.source = native::ring_start,
                                   .destination = destination,
                                   .control = conflict ? copy_x_to_p : native::word_copy}});
  f.machine.bus().write({.space = DspSpace::x, .address = native::ring_start, .value = payload});
  f.machine.bus().write({.space = space, .address = destination, .value = sentinel});
  attach(f);
  const auto initial = f.machine.cpu().registers();
  const auto progress = f.machine.cpu().progress();
  const auto ticks = f.machine.clock().elapsed_ticks();
  static_cast<void>(advance(f));
  const auto sampled = advance(f, 1);
  require(sampled.dma.phase && sampled.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !sampled.dma.core_stall && !sampled.dma.word,
          "The real DMA source phase shares the held native DO setup");

  const auto waited = advance(f, 0, DspDmaGrant::wait);
  require(waited.dma.core_stall == conflict && !waited.dma.word &&
              f.machine.bus().read({.space = space, .address = destination}) == sentinel,
          "Actual conflicting P DMA waits retain both loop ownership and the unwritten cell");
  const auto committed = advance(f);
  require(committed.dma.core_stall == conflict && committed.dma.word &&
              committed.dma.word->value == payload &&
              f.machine.bus().read({.space = space, .address = destination}) == payload,
          "Only the real DMA destination phase commits its independently expected word");
  finish(f, progress.instructions + instructions);
  const auto expected_periods = total_periods + (conflict ? 2U : 0U);
  require(f.machine.cpu().registers() == finished(initial, {.count = count, .width = width}) &&
              f.machine.cpu().progress().estimated_cycles ==
                  progress.estimated_cycles + total_periods &&
              f.machine.clock().elapsed_ticks() == ticks + expected_periods * period,
          "Native loops add only the actual conflicting DMA periods to their native budget");
}

void nested() {
  std::array<std::uint32_t, padding_words> code{};
  code[0] = dor_two; // DOR #2; DOR #2 over two NOPs; one outer tail NOP.
  code[1] = 6;
  code[2] = dor_two;
  code[3] = 3;
  fixture::Fixture f(code);
  attach(f);
  const auto initial = f.machine.cpu().registers();
  const auto progress = f.machine.cpu().progress();
  const auto ticks = f.machine.clock().elapsed_ticks();
  finish(f, progress.instructions + instructions);
  auto expected = finished(initial, {.count = 2, .width = 5});
  constexpr unsigned child_body = body + 2;
  constexpr unsigned outer_last = body + 4;
  expected.stack[3] = (std::uint64_t{outer_last} << word_bits) | 1U;
  expected.stack[4] = (std::uint64_t{child_body} << word_bits) | initial.sr | loop_flag;
  constexpr unsigned native_periods = 25;
  require(
      f.machine.cpu().registers() == expected &&
          f.machine.cpu().progress().estimated_cycles ==
              progress.estimated_cycles + native_periods &&
          f.machine.clock().elapsed_ticks() >= ticks + native_periods * period,
      "Conservative nested-loop refill preserves every native stack, count and restoration field");
}

void repeated_body() {
  std::array<std::uint32_t, padding_words> code{};
  code[0] = dor_two; // DOR #2; REP #3 NOP; two ordinary tail NOPs.
  code[1] = 5;
  code[2] = repeat_three;
  fixture::Fixture f(code);
  attach(f);
  const auto initial = f.machine.cpu().registers();
  const auto progress = f.machine.cpu().progress();
  const auto ticks = f.machine.clock().elapsed_ticks();
  finish(f, progress.instructions + instructions);
  constexpr unsigned native_periods = 25;
  require(
      f.machine.cpu().registers() == finished(initial, {.count = 2, .width = width}) &&
          f.machine.cpu().progress().estimated_cycles ==
              progress.estimated_cycles + native_periods &&
          f.machine.clock().elapsed_ticks() == ticks + native_periods * period &&
          !f.machine.cpu().instruction_fetch().latched_opcode,
      "Actual REP latches and temporary LC values restore correctly inside a predicted DO body");
}
} // namespace

void event_checks() {
  dma(false);
  dma(true);
  nested();
  repeated_body();
  std::cout << "Native loop DMA, nested control and REP event checks passed\n";
}
} // namespace blofeld::test::native_loop
