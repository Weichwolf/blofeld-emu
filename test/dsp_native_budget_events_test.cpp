// SPDX-License-Identifier: GPL-3.0-only
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_native_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
namespace fixture = blofeld::test::fetch_native;
namespace native = blofeld::test::dma_test;
using fixture::require;
constexpr auto timing = OwnDspAutomaticTiming::native_cycle_budget;
constexpr unsigned repeat_immediate = 0x0617a0, payload = 0x123456, sentinel = 0x765432;
constexpr unsigned irqa_vector = 0x10, priority = 0xffffff, edge_priority = 6;
constexpr unsigned move_sr = 0x05f439, move_x0 = 0x44f400, x_value = 7;
constexpr unsigned words = 32;
constexpr unsigned program_destination = 0x300, copy_x_to_p = 0x880248;
constexpr std::uint64_t period = 45, reset_ticks = 90;

struct Request {
  unsigned levels = 0;
  DspDmaGrant grant = DspDmaGrant::available;
};

OwnDspFetchCycleResult advance(fixture::Fixture &f, const Request request = {}) {
  const auto last = f.machine.bus().last_dma_cycle();
  return f.fetch->advance_dma_automatic({.cycle = last ? *last + 1 : 0,
                                         .known_requests = 15,
                                         .requests = request.levels,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = request.grant},
                                        timing);
}

void attach(fixture::Fixture &f, const unsigned prefill = 8) {
  f.attach();
  for (unsigned index = 0; index < prefill; ++index) {
    f.fetch->issue();
    f.fetch->capture();
  }
}

void dma_padding_restore(const bool conflict) {
  std::array<std::uint32_t, words> code{};
  code[0] = repeat_immediate;
  const auto destination = conflict ? program_destination : native::destination;
  const auto space = conflict ? DspSpace::p : DspSpace::y;
  fixture::Fixture f(code,
                     {.transfer = {.source = native::ring_start,
                                   .destination = destination,
                                   .control = conflict ? copy_x_to_p : native::word_copy},
                      .timer = true});
  f.machine.bus().write({.space = DspSpace::x, .address = native::ring_start, .value = payload});
  f.machine.bus().write({.space = space, .address = destination, .value = sentinel});
  attach(f, 2);
  const auto start = f.machine.clock().elapsed_ticks();
  const auto before = f.machine.cpu().progress();
  const auto edges = f.machine.clock().timer_clk2_edges();
  static_cast<void>(advance(f));
  const auto sampled = advance(f, {.levels = 1});
  require(sampled.dma.phase && sampled.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !sampled.dma.core_stall && !sampled.dma.word && f.machine.cpu().progress() == before,
          "Actual DMA source sampling shares a pending REP budget without early retirement");
  const auto saved = f.fetch->checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(saved);
    const auto waited = advance(f, {.grant = DspDmaGrant::wait});
    require(waited.dma.core_stall == conflict && !waited.native && !waited.dma.word &&
                f.machine.bus().read({.space = space, .address = destination}) == sentinel,
            "A held DMA destination stalls native work only for its actual F2 memory conflict");
    const auto committed = advance(f);
    require(committed.dma.core_stall == conflict && !committed.native && committed.dma.word &&
                committed.dma.word->value == payload &&
                f.machine.bus().read({.space = space, .address = destination}) == payload,
            "Restored DMA commits its real word with the same independent/conflicting ownership");
    for (unsigned index = 0; index < (conflict ? 2U : 0U); ++index) {
      const auto held = advance(f);
      require(!held.native && f.machine.cpu().progress() == before,
              "The unpaid REP budget remains pending after the completed DMA destination");
    }
    const auto retired = advance(f);
    const unsigned expected_periods = conflict ? 7U : 5U;
    require(
        retired.native && retired.native->retired && retired.native_pc == fixture::program &&
            f.machine.cpu().progress().instructions == before.instructions + 1 &&
            f.machine.cpu().progress().estimated_cycles == before.estimated_cycles + 5 &&
            f.machine.clock().elapsed_ticks() == start + expected_periods * period &&
            f.machine.clock().timer_clk2_edges() - edges == expected_periods / 2,
        "Each restored suffix retains five native periods plus only its actual stalled periods");
  }
}

void interrupt_padding(const bool missing_second) {
  std::array<std::uint32_t, words> code{};
  code[0] = move_sr;
  code[2] = move_x0;
  code[3] = x_value;
  fixture::Fixture f(code);
  f.machine.bus().write({.address = irqa_vector});
  if (!missing_second) { f.machine.bus().write({.address = irqa_vector + 1}); }
  attach(f);
  const auto before = f.machine.cpu().progress().instructions;
  for (unsigned index = 0; index < 2; ++index) { static_cast<void>(advance(f)); }
  require(f.machine.cpu().registers().sr == 0 &&
              f.machine.cpu().progress().instructions == before + 1,
          "Actual MOVEC unmasks the interrupt before the following two-cycle MOVE");
  const auto move = advance(f);
  require(move.native && !move.native->retired,
          "The actual immediate MOVE still owes its second native period");
  f.machine.bus().write({.space = DspSpace::x, .address = priority, .value = edge_priority});
  f.machine.bus().observe_irqa(true);
  f.machine.bus().observe_irqa(false);
  const auto completed = advance(f);
  require(completed.native && completed.native->retired &&
              completed.native_pc == fixture::program + 2 && f.machine.bus().interrupt_request(0) &&
              !f.machine.interrupt_pending(),
          "A newly asserted IRQ cannot replace the pending MOVE's genuine retirement");
  const auto foreground = f.machine.cpu().registers();
  const auto count = f.machine.cpu().progress().instructions;
  const auto first = advance(f);
  require(first.native && first.native->pending &&
              first.native->pending->effect.kind == dsp56300::EffectKind::fetch_extension &&
              first.native->pending->effect.address == irqa_vector + 1 &&
              f.machine.interrupt_pending() && f.machine.bus().interrupt_request(0) &&
              f.machine.cpu().registers() == foreground &&
              f.machine.cpu().progress().instructions == count,
          "The first genuine vector read keeps the IRQ pending before either slot executes");
  const auto second = advance(f);
  if (missing_second) {
    require(second.native && second.native->fault.kind == dsp56300::FaultKind::memory_read &&
                second.native->fault.address == irqa_vector + 1 &&
                f.machine.bus().interrupt_request(0) && f.machine.interrupt_pending() &&
                f.machine.cpu().progress().instructions == count,
            "A missing second vector word retains its contextual fault and unacknowledged IRQ");
    return;
  }
  require(second.native && second.native->retired && second.native_pc == irqa_vector &&
              !f.machine.bus().interrupt_request(0) && !f.machine.interrupt_pending(),
          "Only the actual second vector read permits acknowledgement and first-slot retirement");
  const auto tail = advance(f);
  require(tail.native && tail.native->retired && tail.native_pc == irqa_vector + 1 &&
              f.machine.cpu().registers() == foreground &&
              f.machine.cpu().progress().instructions == count + 2,
          "The actual latched second NOP returns to the untouched original foreground");
}

void reset_padding() {
  std::array<std::uint32_t, words> code{};
  code[0] = repeat_immediate;
  fixture::Fixture f(code);
  attach(f);
  static_cast<void>(advance(f));
  const auto pending = f.machine.cpu().pending_instruction();
  require(pending && pending->effect.kind == dsp56300::EffectKind::retire,
          "The original REP is genuinely waiting at its computed retirement");
  f.machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(!fixture::rejects([&] { static_cast<void>(f.fetch->view()); }).empty(),
          "Actual hardware reset invalidates the old pending-budget fetch owner");
  require(f.machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "The second actual reset completes its asserted hold");
  f.machine.release_hardware_reset();
  require(f.machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "The second actual reset completes recovery before execution");
  f.fetch.reset();
  attach(f);
  const auto first = advance(f);
  require(first.native && !first.native->retired &&
              f.machine.cpu().progress() == dsp56300::Progress{},
          "New reset execution starts the literal two-cycle R0 setup with an unpaid budget");
  const auto second = advance(f);
  require(second.native && second.native->retired && second.native_pc == native::boot &&
              f.machine.cpu().progress() ==
                  dsp56300::Progress{.instructions = 1, .estimated_cycles = 2},
          "The reset continuation cannot inherit an earlier instruction's paid periods");
}
}

int main() {
  try {
    dma_padding_restore(false);
    dma_padding_restore(true);
    interrupt_padding(false);
    interrupt_padding(true);
    reset_padding();
    std::cout << "Native budgets preserve actual DMA, interrupt vector reads, reset and restore\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
