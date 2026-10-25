// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using blofeld::test::require;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned nop_ticks = 45;
constexpr unsigned twice = 2 * nop_ticks;

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates};
}

blofeld::DspMachine &initialize(blofeld::DspMachine &machine) {
  blofeld::test::DspProgram program;
  for (unsigned i = 0; i < 4; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  return machine;
}

struct Fixture {
  blofeld::DspMachine machine{contract()};
  blofeld::DspScheduledRun runner{
      {.observation = blofeld::DspStepObservation::reference_completion_boundary},
      initialize(machine)};
};

void test_committed_order_and_replay() {
  Fixture f;
  const auto cpu = f.machine.checkpoint();
  const auto schedule = f.runner.checkpoint(f.machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.machine.restore(cpu);
    f.runner.restore(schedule, f.machine);
    std::vector<std::uint64_t> observations;
    const auto before = [&](const blofeld::DspScheduledStep &step) {
      observations.push_back(f.machine.run_clock().elapsed_ticks());
      require(step.plan.start_ticks == f.machine.run_clock().elapsed_ticks(),
              "Before callback sees only the completed prefix preceding its instruction");
    };
    const blofeld::DspCompletionObserver after{.observe = [&](const auto &step) {
      const auto ticks = f.machine.run_clock().elapsed_ticks();
      observations.push_back(ticks);
      require(ticks == step.completion_ticks && ticks == step.plan.start_ticks + nop_ticks &&
                  f.machine.run_clock().native_cycle_units() == ticks / nop_ticks &&
                  f.runner.completed_steps() == ticks / nop_ticks &&
                  f.machine.cpu().state().pc == blofeld::test::boot + ticks / nop_ticks,
              "After callback sees native NOP PC, clock and committed completion accounting");
      static_cast<void>(
          fixture::rejects([&] { static_cast<void>(f.runner.checkpoint(f.machine)); },
                           "An active post-completion callback cannot capture the schedule"));
      static_cast<void>(fixture::rejects(
          [&] { f.runner.advance_to(f.machine, twice, blofeld::DspScheduleBoundary::through, 2); },
          "An active post-completion callback cannot reenter its schedule"));
    }};
    require(f.runner.advance_to(
                f.machine, twice, blofeld::DspScheduleBoundary::through, 2, before, after) == 2 &&
                observations == std::vector<std::uint64_t>{0, nop_ticks, nop_ticks, twice},
            "Each before/after pair surrounds exactly one native one-unit NOP in both replays");
  }
}

void test_failed_observer_retains_completed_prefix_and_recovers() {
  Fixture f;
  const auto cpu = f.machine.checkpoint();
  const auto schedule = f.runner.checkpoint(f.machine);
  const blofeld::DspCompletionObserver fail{.observe = [](const auto & /*step*/) {
    throw blofeld::Fault("Own failed external signal observation");
  }};
  const auto error = fixture::rejects(
      [&] {
        f.runner.advance_to(f.machine, twice, blofeld::DspScheduleBoundary::through, 2, {}, fail);
      },
      "An external observation error cannot turn into a successful requested horizon");
  require(f.runner.faulted() && f.runner.completed_steps() == 1 && f.runner.horizon_ticks() == 0 &&
              f.machine.run_clock().elapsed_ticks() == nop_ticks &&
              f.machine.cpu().state().pc == blofeld::test::boot + 1 &&
              !f.machine.timing_faulted() && error.contains("post-completion observation at=45") &&
              error.contains("completed-steps=1") && error.contains("completed-reference-ticks=45"),
          "The completed native instruction stays committed while the failed schedule quarantines");
  static_cast<void>(fixture::rejects(
      [&] { f.runner.advance_to(f.machine, twice, blofeld::DspScheduleBoundary::through, 2); },
      "A failed observer requires coordinated recovery before scheduling continues"));
  f.machine.restore(cpu);
  f.runner.restore(schedule, f.machine);
  require(f.runner.advance_to(f.machine, twice, blofeld::DspScheduleBoundary::through, 2) == 2,
          "Coordinated recovery reexecutes the independent two-NOP prefix without a hidden skip");
}

void test_clock_mutation_and_callback_lifetime() {
  Fixture f;
  const auto cpu = f.machine.checkpoint();
  const auto schedule = f.runner.checkpoint(f.machine);
  const blofeld::DspCompletionObserver mutate{
      .observe = [&](const auto & /*step*/) { f.machine.cpu().step(); }};
  static_cast<void>(fixture::rejects(
      [&] {
        f.runner.advance_to(f.machine, twice, blofeld::DspScheduleBoundary::through, 2, {}, mutate);
      },
      "A post-completion callback must not advance the native owner behind its schedule"));
  require(f.runner.faulted() && f.runner.completed_steps() == 1 &&
              f.machine.run_clock().elapsed_ticks() == twice,
          "A real extra NOP is detected before another scheduled instruction can execute");
  f.machine.restore(cpu);
  f.runner.restore(schedule, f.machine);
  blofeld::DspCompletionObserver replace;
  unsigned calls = 0;
  constexpr unsigned marker_size = 64;
  constexpr unsigned marker_value = 23;
  std::array<unsigned, marker_size> marker{};
  marker.fill(marker_value);
  replace.observe = [&, marker](const auto & /*step*/) {
    replace.observe = {};
    require(marker.back() == marker_value, "A cleared source callback retains its active copy");
    ++calls;
  };
  require(f.runner.advance_to(
              f.machine, twice, blofeld::DspScheduleBoundary::through, 2, {}, replace) == 2 &&
              calls == 1,
          "Replacing an external callback is safe and takes effect for the next completion");
}
}

int main() {
  try {
    test_committed_order_and_replay();
    test_failed_observer_retains_completed_prefix_and_recovers();
    test_clock_mutation_and_callback_lifetime();
    std::cout << "Committed DSP observations, failure recovery and callback lifetime passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
