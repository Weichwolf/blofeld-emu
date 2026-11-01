// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_fetch_reset_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

namespace blofeld::test::fetch_reset {
OwnDspResetInputs pins() {
  return {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3};
}

DspDmaInputs inputs(OwnDspMachine &machine) {
  const auto last = machine.bus().last_dma_cycle();
  return {.cycle = last ? *last + 1 : 0,
          .known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

Fixture::Fixture(const Start start) {
  if (start == Start::blank) { return; }
  // FM immediate MOVE, CLR A, ADD X0,A and absolute JMP. No assembler dependency.
  constexpr std::array<std::uint32_t, 32> program{immediate_x0, 1, clear_a, add_x0, jump, boot + 3};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
  power_on();
}

void Fixture::power_on() {
  machine.bus().write({.space = DspSpace::x, .address = ram, .value = marker});
  reset();
  held(2 * period, 2 * period);
  timeline->release_hardware_reset();
  held(recovery_ticks, recovery_ticks);
  trace.clear();
}

void Fixture::time() const {
  const auto &clock = machine.clock();
  require(clock.native_cycle_units() == ledger.cycles && clock.elapsed_ticks() == ledger.ticks &&
              clock.hardware_reset_ticks() == ledger.reset_ticks &&
              clock.timer_clk2_edges() == ledger.clk2 && clock.native_epoch() == ledger.epoch &&
              clock.epoch_native_cycle_units() == ledger.cycles - ledger.epoch_start &&
              clock.interlock_cycle_units() == 0 &&
              ledger.ticks == (ledger.cycles * period) + ledger.reset_ticks,
          "Literal lifetime/native/reset/CLK2 ledgers retain both real reset and execution time");
}

Observation Fixture::observe(const OwnDspFetchCycleResult *const cycle) {
  const auto pending = machine.cpu().pending_instruction();
  return {.registers = machine.cpu().registers(),
          .progress = machine.cpu().progress(),
          .fault = machine.cpu().fault(),
          .pending = pending ? std::optional{pending->effect} : std::nullopt,
          .outcome = machine.cpu().instruction_outcome(),
          .reset = machine.clock().hardware_reset_state(),
          .dma = machine.bus().last_dma_cycle(),
          .issued = cycle != nullptr ? cycle->issued_address : std::nullopt,
          .ticks = machine.clock().elapsed_ticks(),
          .cycles = machine.clock().native_cycle_units(),
          .clk2 = machine.clock().timer_clk2_edges(),
          .epoch = machine.clock().native_epoch(),
          .captured = cycle != nullptr && cycle->captured,
          .retired = cycle != nullptr && cycle->native && cycle->native->retired};
}

void Fixture::reset() {
  auto expected = machine.cpu().registers();
  expected.pc = boot;
  expected.sr = reset_status;
  expected.omr = reset_omr;
  expected.sp = 0;
  expected.sc = 0;
  expected.vba = 0;
  expected.m.fill(dsp56300::word_mask);
  if (timeline) {
    timeline->begin_hardware_reset(pins());
  } else {
    timeline = std::make_unique<OwnDspTimeline>(machine, timing, 0, pins());
  }
  ++ledger.epoch;
  ledger.epoch_start = ledger.cycles;
  ledger.adds = 0;
  ledger.instructions = 0;
  ledger.cost = 0;
  ledger.pc = boot;
  ledger.expected = expected;
  require(machine.clock().hardware_reset_active() && machine.cpu().registers() == expected &&
              machine.cpu().progress() == dsp56300::Progress{} &&
              !machine.cpu().pending_instruction() && !machine.cpu().instruction_outcome() &&
              !machine.bus().last_dma_cycle() && machine.bus().pll().read() == reset_pctl &&
              machine.bus().read({.space = DspSpace::x, .address = ram}) == marker,
          "Actual native reset flushes work, resets only documented fields and retains RAM");
  time();
}

void Fixture::held(const std::uint64_t requested, const std::uint64_t expected) {
  const auto before = observe();
  require(timeline->advance_reset_to(ledger.ticks + requested) == expected,
          "Reset consumes the independently supplied duration and clamps at recovery completion");
  ledger.ticks += expected;
  ledger.reset_ticks += expected;
  require(machine.cpu().registers() == before.registers &&
              machine.cpu().progress() == before.progress && !machine.bus().last_dma_cycle(),
          "Held/recovering EXTAL time executes no CPU instruction or DMA transaction");
  time();
}

OwnDspFetchCycleResult Fixture::step() {
  require(timeline->next_deadline() == ledger.ticks + period,
          "Actual next deadline follows the fixed bypass period without a reset-time hole");
  auto result =
      timeline->advance_one(timeline->next_deadline(), OwnDspBoundary::through, inputs(machine));
  if (!result) { throw Fault("Next deadline did not execute its actual native reference period"); }
  require(!result->native || result->native->fault.kind == dsp56300::FaultKind::none,
          "Actual native reset suffix remains healthy");
  ledger.clk2 += (ledger.cycles - ledger.epoch_start) & 1U;
  ++ledger.cycles;
  ledger.ticks += period;
  time();
  const auto epoch_cycle = ledger.cycles - ledger.epoch_start;
  if (epoch_cycle <= 2) {
    require(!result->native && !result->instruction_started &&
                result->captured == (epoch_cycle == 2) &&
                result->issued_address == boot + static_cast<unsigned>(epoch_cycle - 1),
            "A fresh epoch starts with real F1, then F2; no abandoned instruction resumes");
  }
  trace.push_back(observe(&*result));
  return *result;
}

void Fixture::oracle() {
  const auto outcome = machine.cpu().instruction_outcome();
  if (!outcome) { throw Fault("Actual native retirement lost its outcome"); }
  require(outcome->address == ledger.pc, "Retirement follows literal MOVE/CLR/ADD/JMP addresses");
  unsigned opcode = 0;
  unsigned cost = 1;
  constexpr unsigned zero = 4;
  constexpr unsigned unnormalized = 16;
  auto &expected = ledger.expected;
  if (ledger.pc == boot) {
    opcode = immediate_x0;
    cost = 2;
    expected.x = 1;
    ledger.pc += 2;
  } else if (ledger.pc == boot + 2) {
    opcode = clear_a;
    expected.a = 0;
    expected.sr =
        reset_status | zero | unnormalized; // FM: zero, unnormalized, no extension/overflow/carry.
    ++ledger.pc;
  } else if (ledger.pc == boot + 3) {
    opcode = add_x0;
    ++ledger.adds;
    expected.a = ledger.adds << accumulator_bits;
    expected.sr =
        reset_status | unnormalized; // Positive small fractional sum remains unnormalized.
    ++ledger.pc;
  } else {
    require(ledger.pc == boot + 4, "No fabricated arithmetic-loop retirement");
    opcode = jump;
    cost = 4; // FM table A-1: T=3 plus one long-absolute addressing cycle.
    ledger.pc = boot + 3;
  }
  expected.pc = ledger.pc;
  ++ledger.instructions;
  ledger.cost += cost;
  require(
      outcome->opcode == opcode && machine.cpu().registers() == expected &&
          machine.cpu().progress() == dsp56300::Progress{.instructions = ledger.instructions,
                                                         .estimated_cycles = ledger.cost},
      "Full native registers, flags, exact arithmetic and documented base costs match literals");
}

void Fixture::advance(const unsigned count) {
  for (unsigned i = 0; i < count; ++i) {
    const auto result = step();
    if (result.native && result.native->retired) { oracle(); }
  }
}

void Fixture::restore(const OwnDspTimelineCheckpoint &checkpoint, const Ledger &saved) {
  const auto previous = std::move(timeline);
  timeline = std::make_unique<OwnDspTimeline>(machine, checkpoint);
  ledger = saved;
  trace.clear();
  if (previous) {
    static_cast<void>(rejects([&] { static_cast<void>(previous->checkpoint()); }));
  }
  time();
}

void Fixture::gated() {
  const auto before = observe();
  static_cast<void>(rejects([&] { static_cast<void>(machine.begin_fetch()); }));
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance_reference_cycle()); }));
  static_cast<void>(
      rejects([&] { static_cast<void>(machine.advance_dma_reference_cycle(inputs(machine))); }));
  static_cast<void>(rejects([&] { static_cast<void>(timeline->next_deadline()); }));
  static_cast<void>(rejects([&] {
    static_cast<void>(
        timeline->advance_one(ledger.ticks + period, OwnDspBoundary::through, inputs(machine)));
  }));
  require(observe() == before && !timeline->faulted(),
          "Actual native/DMA/timeline admissions refuse under reset without consuming their gate");
  time();
}
}
