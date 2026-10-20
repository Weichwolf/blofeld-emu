// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <memory>
#include <vector>

namespace blofeld::test::native_access {
namespace {
void missing_extension() {
  DspMachine machine;
  constexpr std::array words{immediate_x0};
  machine.bus().install_rom({.start = boot, .words = words});
  machine.cpu().reset_spi_boot();
  const auto frozen = machine.checkpoint();
  Recorder capture;
  const auto error = fault([&] { capture.step(machine); });
  require(error.contains("pc=0xff0000") && error.contains("P:0xff0001") &&
              capture.rows.size() == 3 && capture.rows.back().kind == Kind::extension &&
              capture.rows.back().stage == Stage::before && !capture.rows.back().value &&
              machine.cpu().debug_state().native_instructions == 0,
          "An unknown consumed extension is an attempted strict fetch, not a zero operand");
  const auto previous = capture.rows.size();
  static_cast<void>(fault([&] { machine.cpu().step(capture.observer()); }));
  require(capture.rows.size() == previous, "Rejected retry emits no speculative native access");
  machine.restore(frozen);
  constexpr std::array replacement{first};
  machine.bus().install_rom({.start = boot + 1, .words = replacement});
  capture.step(machine);
  require(capture.rows.size() == 4 && machine.cpu().state().x == first,
          "Trusted restore plus an explicit known extension recovers actual native execution");

  DspMachine unknown;
  DspProgram code;
  code.emit("move x:>$100,x0");
  code.install(unknown.bus());
  unknown.cpu().reset_spi_boot();
  const auto data_error = fault([&] { capture.step(unknown); });
  require(data_error.contains("Unknown DSP memory read at X:0x000100") &&
              capture.rows.size() == 5 && capture.rows.back().kind == Kind::read &&
              capture.rows.back().target.space == DspSpace::x &&
              capture.rows.back().target.address == data && !capture.rows.back().value,
          "An unknown data source retains the completed fetch prefix and no successful value");
}

void callback_failure_prefixes() {
  const DspRunContract timing{
      .clock = {.extal_hz = 12000000,
                .timebase_hz = 540000000,
                .switch_timing = DspClockSwitch::reference_instruction_boundary},
      .timing = DspRunTiming::native_internal_estimates};
  for (const auto kind : {Kind::opcode, Kind::extension, Kind::write}) {
    for (const auto stage : {Stage::before, Stage::after}) {
      DspMachine machine(timing);
      DspProgram code;
      code.immediate("x0", first);
      const auto store_pc = code.pc();
      code.emit("move x0,x:>$100");
      code.install(machine.bus());
      machine.bus().pll().observe_pinit(false);
      machine.cpu().reset_spi_boot();
      machine.bus().write({.space = DspSpace::x, .address = data, .value = second});
      machine.cpu().step();
      const auto frozen = machine.checkpoint();
      Recorder capture;
      const DspNativeAccessObserver reject{.observe = [&](const DspNativeAccess &event) {
        capture.rows.push_back(event);
        if (event.kind == kind && event.stage == stage) {
          throw Fault("Own native access observer failure");
        }
      }};
      const auto error = fault([&] { machine.cpu().step(reject); });
      const bool stored = kind == Kind::write && stage == Stage::after;
      constexpr unsigned completed_units = 2;
      constexpr unsigned prefix_ticks = 90;
      require(error.contains("pc=0xff0002") && error.contains("DSP native access") &&
                  error.contains("Own native access observer failure") &&
                  machine.bus().read({.space = DspSpace::x, .address = data}) ==
                      (stored ? first : second) &&
                  machine.timing_faulted() &&
                  machine.run_clock().native_cycle_units() == completed_units &&
                  machine.run_clock().elapsed_ticks() == prefix_ticks &&
                  machine.cpu().debug_state().native_instructions == 1,
              "Observer failure quarantines the unretired step and retains only real bus effects");
      static_cast<void>(fault([&] { machine.cpu().step(); }));
      static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
      machine.restore(frozen);
      capture.step(machine);
      constexpr unsigned total_units = 4;
      constexpr unsigned total_ticks = 180;
      require(capture.rows.size() == 6 && machine.cpu().state().pc == store_pc + 2 &&
                  machine.bus().read({.space = DspSpace::x, .address = data}) == first &&
                  machine.run_clock().native_cycle_units() == total_units &&
                  machine.run_clock().elapsed_ticks() == total_ticks,
              "Coordinated recovery emits a fresh complete trace and exactly one native store");
    }
  }
}

void lifetime_and_reentry() {
  DspMachine machine;
  DspProgram code;
  code.emit("nop");
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  const auto frozen = machine.checkpoint();
  auto held = std::make_shared<unsigned>(0);
  const std::weak_ptr<unsigned> weak = held;
  unsigned calls = 0;
  DspNativeAccessObserver observer;
  observer.observe = [owned = held, &observer, &machine, &frozen, &calls](const auto & /*event*/) {
    ++*owned;
    ++calls;
    observer.observe = {}; // The running callback must own a separate value copy.
    static_cast<void>(fault([&] { machine.cpu().step(); }));
    static_cast<void>(fault([&] { machine.cpu().reset_spi_boot(); }));
    static_cast<void>(fault([&] { machine.restore(frozen); }));
    static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  };
  held.reset();
  machine.cpu().step(observer);
  require(calls == 2 && weak.expired() && machine.cpu().state().pc == boot + 1,
          "One owned callback survives source clearing, rejects reentry and ends with its step");
  machine.cpu().step();
  require(calls == 2 && machine.cpu().state().pc == boot + 2,
          "A later default step retains no callback or observer cursor");
}

void non_standard_failure() {
  DspMachine machine;
  DspProgram code;
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  const auto frozen = machine.checkpoint();
  const DspNativeAccessObserver reject{.observe = [](const DspNativeAccess &event) {
    if (event.stage == Stage::after) { throw 7; }
  }};
  const auto error = fault([&] { machine.cpu().step(reject); });
  require(error.contains("after opcode at P:0xff0000 ordinal=0") &&
              error.contains("Non-standard native access observer exception") &&
              machine.cpu().debug_state().native_instructions == 0,
          "A non-standard callback exception cannot escape without native quarantine/context");
  static_cast<void>(fault([&] { machine.cpu().step(); }));
  machine.restore(frozen);
  machine.cpu().step();
  require(machine.cpu().debug_state().native_instructions == 1,
          "Trusted restore recovers a non-standard callback failure without retaining it");
}
}

void failure_checks() {
  missing_extension();
  callback_failure_prefixes();
  lifetime_and_reentry();
  non_standard_failure();
}
}
