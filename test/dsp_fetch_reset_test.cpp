// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_reset_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

namespace blofeld::test::fetch_reset {
namespace {
unsigned reset_case(const unsigned prefix) {
  Fixture source;
  source.advance(prefix);
  const auto pending = source.machine.cpu().pending_instruction();
  const auto stale = pending ? std::optional{pending->ticket} : std::nullopt;
  unsigned kind = 0;
  if (source.trace.size() > 1) {
    const auto &before = source.trace[source.trace.size() - 2];
    const auto &after = source.trace.back();
    if (before.registers.x != after.registers.x) {
      kind |= 1U << static_cast<unsigned>(dsp56300::EffectKind::move_update);
    }
    if (before.registers.a != after.registers.a || before.registers.sr != after.registers.sr) {
      kind |= 1U << static_cast<unsigned>(dsp56300::EffectKind::alu_update);
    }
    if (before.registers.pc != after.registers.pc) {
      kind |= 1U << static_cast<unsigned>(dsp56300::EffectKind::pcu_update);
    }
    if (before.progress.instructions != after.progress.instructions) {
      kind |= 1U << static_cast<unsigned>(dsp56300::EffectKind::retire);
    }
  }
  source.reset();
  source.gated();
  source.held(held_ticks, held_ticks);
  const auto state = source.machine.clock().hardware_reset_state();
  require(state && state->phase == DspHardwareResetPhase::asserted &&
              state->asserted_extal_cycles == 2 && state->phase_ticks == 1,
          "91 ticks retain two whole45tick EXTAL periods and a one-tick fraction");
  const auto checkpoint = source.timeline->checkpoint();
  const auto ledger = source.ledger;
  Fixture fresh(Start::blank);
  std::vector<Observation> expected;
  for (auto *const target : {&source, &fresh}) {
    target->restore(checkpoint, ledger);
    target->timeline->release_hardware_reset();
    target->gated();
    require(target->machine.clock().hardware_reset_remaining_ticks() == recovery_ticks,
            "Release starts three supplied bypass EXTAL periods without carrying hold residue");
    target->held(recovery_ticks - 1, recovery_ticks - 1);
    target->gated();
    target->held(period, 1);
    require(!target->machine.clock().hardware_reset_active() &&
                target->machine.clock().timer_clk2_edges() == ledger.clk2,
            "Recovery clamps at its endpoint before any native F1/F2 or CLK2 edge");
    if (stale) {
      const auto before = target->observe();
      static_cast<void>(rejects([&] { static_cast<void>(target->machine.advance(*stale)); }));
      require(target->observe() == before, "Abandoned effect cannot mutate either restored owner");
    }
    target->advance(suffix_cycles);
    require(target->ledger.adds > 0, "Actual reset entry executes literal native arithmetic");
    if (target == &source) {
      expected = target->trace;
    } else {
      require(
          target->trace == expected,
          "Fresh chip/frontend/clock restore repeats every native effect and fetch observation");
    }
  }
  return kind;
}

void recovery_checkpoint() {
  Fixture f;
  f.advance(7);
  f.reset();
  f.held(held_ticks, held_ticks);
  f.timeline->release_hardware_reset();
  f.held(partial_recovery, partial_recovery);
  const auto checkpoint = f.timeline->checkpoint();
  const auto ledger = f.ledger;
  const auto observed = f.observe();
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    f.restore(checkpoint, ledger);
    require(f.observe() == observed &&
                f.machine.clock().hardware_reset_remaining_ticks() == held_ticks,
            "Actual joint recovery snapshot retains its44tick EXTAL phase and boot gate");
    f.held(recovery_ticks, held_ticks);
    f.advance(suffix_cycles);
    f.reset();
    f.held(2 * period, 2 * period);
    f.timeline->release_hardware_reset();
    f.held(recovery_ticks, recovery_ticks);
    f.advance(suffix_cycles);
  }
}

void pin_guard(const unsigned kind) {
  Fixture f;
  f.advance(7);
  auto invalid = pins();
  if (kind == 0) { invalid.mode_known = 7; }
  if (kind == 1) { invalid.mode_levels = 4; }
  if (kind == 2) { invalid.pinit.reset(); }
  if (kind == 3) { invalid.pinit = true; }
  if (kind == 4) { invalid.recovery_cycles = 1; }
  const auto before = f.observe();
  static_cast<void>(rejects([&] { f.timeline->begin_hardware_reset(invalid); }));
  require(f.observe() == before && !f.timeline->faulted(),
          "Unsupported pins/recovery refuse before native reset or frontend invalidation");
  f.advance(1);
}

void phase_guards() {
  Fixture f;
  f.advance(1);
  const auto before = f.observe();
  require(!f.timeline->advance_one(
              f.ledger.ticks + period - 1, OwnDspBoundary::through, inputs(f.machine)),
          "An observation before the next deadline advances no fabricated native period");
  static_cast<void>(rejects([&] { f.timeline->begin_hardware_reset(pins()); }));
  require(f.observe() == before && !f.timeline->faulted(),
          "A future observation cannot assert reset retroactively at an earlier completed period");
  f.advance(1);
  f.reset();
  f.held((2 * period) - 1, (2 * period) - 1);
  const auto held = f.observe();
  static_cast<void>(rejects([&] { f.timeline->release_hardware_reset(); }));
  static_cast<void>(rejects([&] { f.timeline->begin_hardware_reset(pins()); }));
  static_cast<void>(rejects([&] { static_cast<void>(f.machine.advance_hardware_reset_ticks(0)); }));
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  static_cast<void>(
      rejects([&] { static_cast<void>(f.machine.advance_hardware_reset_ticks(maximum)); }));
  require(f.timeline->advance_reset_to(f.ledger.ticks) == 0 && f.observe() == held &&
              !f.timeline->faulted(),
          "Early release/reassertion/duration overflow refuse; a zero-time observation is inert");
  f.time();
  f.held(1, 1);
  f.timeline->release_hardware_reset();
  f.held(recovery_ticks, recovery_ticks);
  f.advance(suffix_cycles);
}
}

void guard_checks() {
  for (unsigned kind = 0; kind < 5; ++kind) { pin_guard(kind); }
  Fixture f;
  const auto before = f.observe();
  static_cast<void>(rejects([&] {
    const OwnDspTimeline invalid(f.machine, OwnDspAutomaticTiming::unqualified, 0, pins());
  }));
  require(f.observe() == before && !f.timeline->faulted(),
          "Unqualified timing cannot acquire an existing native owner or assert reset");
  phase_guards();
}
}

int main() {
  using namespace blofeld::test::fetch_reset;
  try {
    unsigned kinds = 0;
    for (unsigned prefix = 0; prefix < cuts; ++prefix) { kinds |= reset_case(prefix); }
    for (const auto kind : {dsp56300::EffectKind::move_update,
                            dsp56300::EffectKind::alu_update,
                            dsp56300::EffectKind::pcu_update,
                            dsp56300::EffectKind::retire}) {
      require((kinds & (1U << static_cast<unsigned>(kind))) != 0,
              "The reset matrix actually reaches MOVE/ALU/PCU/retirement boundaries");
    }
    recovery_checkpoint();
    guard_checks();
    store_checks();
    std::cout << "Native fetch/reset:20 cycle cuts,40 same/fresh suffixes, recovery/guards and4 "
                 "staged long-store cuts passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
