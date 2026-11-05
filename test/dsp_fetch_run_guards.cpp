// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp_fetch_native_fixture.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace blofeld::test::fetch_native {
namespace {
void unknown_pinit() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram code;
  code.word(0);
  code.install(machine.bus());
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  static_cast<void>(rejects([&] { const OwnDspFetch fetch(machine); }));
  const auto error = rejects([&] {
    machine.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = std::nullopt, .recovery_cycles = 2});
  });
  require(error.contains("stable mode5/known PINIT") && error.contains("pc=" + hex(before.pc, 6)) &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress &&
              machine.clock().elapsed_ticks() == 0 && !machine.bus().last_dma_cycle(),
          "Unknown PINIT/reset contract refuses before frontend, native, DMA or clock mutation");
}

void invalid_period_failure() {
  const std::array<std::uint32_t, 16> words{};
  auto clocks = native::native_clocks;
  clocks.timebase_hz = clocks.extal_hz;
  Fixture fixture(words, {.clocks = clocks});
  auto timeline = std::make_unique<OwnDspTimeline>(fixture.machine, timing, 0);
  const auto first_deadline = timeline->next_deadline();
  require(
      timeline->advance_one(first_deadline, OwnDspBoundary::through, banks::inputs({.index = 1}))
          .has_value(),
      "Integral one-tick period admits one actual F1/reference cycle");
  const auto attempted = timeline->next_deadline();
  fixture.machine.bus().pll().write(faster);
  const auto before = fixture.machine.cpu().registers();
  const auto progress = fixture.machine.cpu().progress();
  const auto error = rejects([&] {
    static_cast<void>(
        timeline->advance_one(attempted, OwnDspBoundary::through, banks::inputs({.index = 2})));
  });
  require(error.contains("not integral") && timeline->faulted() && fixture.elapsed() == 1 &&
              fixture.machine.bus().last_dma_cycle() == 1 &&
              fixture.machine.cpu().registers() == before &&
              fixture.machine.cpu().progress() == progress,
          "Nonintegral period quarantines attempted timeline execution without rounding, DMA or "
          "CPU effects");
  static_cast<void>(rejects([&] {
    static_cast<void>(
        timeline->advance_one(attempted, OwnDspBoundary::through, banks::inputs({.index = 2})));
  }));
  require(fixture.elapsed() == 1 && fixture.machine.bus().last_dma_cycle() == 1 &&
              fixture.machine.cpu().registers() == before &&
              fixture.machine.cpu().progress() == progress,
          "A quarantined period cannot resume execution or repeat device effects");
  Fixture valid(words, {.clocks = clocks});
  OwnDspTimeline integral(valid.machine, timing, 0);
  for (unsigned index = 1; index <= 2; ++index) {
    require(integral
                .advance_one(integral.next_deadline(),
                             OwnDspBoundary::through,
                             banks::inputs({.index = index}))
                .has_value(),
            "An independently reset owner retains its genuine integral period");
  }
  require(valid.elapsed() == 2, "Two valid reference cycles advance exactly two ticks");
}

void owner_and_contract_guards() {
  const std::array<std::uint32_t, 16> words{};
  Fixture fixture(words);
  Fixture foreign(words);
  static_cast<void>(rejects([&] {
    const OwnDspTimeline invalid(fixture.machine, OwnDspAutomaticTiming::unqualified, 0);
  }));
  auto timeline = std::make_unique<OwnDspTimeline>(fixture.machine, timing, 0);
  static_cast<void>(rejects([&] { const OwnDspTimeline duplicate(fixture.machine, timing, 0); }));
  OwnDspTimeline clone(foreign.machine, timing, 0);
  const auto result = clone.advance_one(
      clone.next_deadline(), OwnDspBoundary::through, banks::inputs({.index = 1}));
  require(result && result->issued_address == program && foreign.elapsed() == base_period &&
              fixture.elapsed() == 0 && fixture.machine.bus().last_dma_cycle() == 0,
          "A fresh owner permits a real independent continuation without mutating its source");
  static_cast<void>(native::dma(fixture.machine, banks::inputs({.index = 1}), banks::use()));
  static_cast<void>(rejects([&] { static_cast<void>(timeline->next_deadline()); }));
  require(fixture.elapsed() == 0, "Out-of-band DMA cannot create an unobserved clock hole");
}

void completion_owner() {
  auto clocks = native::native_clocks;
  clocks.switch_timing = DspClockSwitch::reference_instruction_boundary;
  const std::array words{0U};
  Fixture fixture(words, {.clocks = clocks});
  const auto before = fixture.machine.cpu().registers();
  const auto progress = fixture.machine.cpu().progress();
  static_cast<void>(rejects([&] { const OwnDspTimeline invalid(fixture.machine, timing, 0); }));
  require(fixture.machine.cpu().registers() == before &&
              fixture.machine.cpu().progress() == progress && fixture.elapsed() == 0 &&
              fixture.machine.bus().last_dma_cycle() == 0,
          "A reference-cycle timeline cannot acquire an instruction-boundary clock contract");
  native::step_native(fixture.machine);
  auto expected = before;
  ++expected.pc;
  require(fixture.machine.cpu().registers() == expected &&
              fixture.machine.cpu().progress().instructions == progress.instructions + 1 &&
              fixture.elapsed() == 0,
          "Rejected timeline acquisition preserves actual native execution and its existing clock "
          "contract");
}
}

void guard_checks() {
  unknown_pinit();
  invalid_period_failure();
  owner_and_contract_guards();
  completion_owner();
}
}
