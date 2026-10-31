// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr Word boot_entry = 0xff0000;
constexpr Word destination = 0x10;
constexpr Word immediate_x0 = 0x44f400, clear_a = 0x200013;
constexpr Word repeat_three = 0x0603a0, add_x0 = 0x200040, store_short = 0x441000;
constexpr std::array<Word, 12> program{
    immediate_x0, 7, clear_a, repeat_three, add_x0, store_short, 0};
constexpr auto timing =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
constexpr OwnDspResetInputs reset{
    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3};
constexpr unsigned hold_ticks = 90, recovery_end = 225, instructions = 8;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

void rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &) { return; }
  throw std::runtime_error("Expected checkpoint refusal");
}

void install(OwnDspMachine &machine) {
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = program});
}

void advance(OwnDspMachine &machine, OwnDspTimeline &timeline) {
  const auto cycle = machine.bus().last_dma_cycle();
  const auto result = timeline.advance_one(timeline.next_deadline(),
                                           OwnDspBoundary::through,
                                           {.cycle = cycle ? *cycle + 1 : 0,
                                            .known_requests = 15,
                                            .timing = DspDmaTiming::reference_sampled_cycles,
                                            .grant = DspDmaGrant::available});
  if (!result) {
    throw std::runtime_error("Actual next deadline did not execute a reference cycle");
  }
  require(!result->native || result->native->fault.kind == FaultKind::none,
          "Restored instruction produced a native fault");
}

void finish(OwnDspMachine &machine, OwnDspTimeline &timeline) {
  constexpr unsigned bound = 128;
  for (unsigned cycle = 0; cycle < bound && machine.cpu().progress().instructions < instructions;
       ++cycle) {
    advance(machine, timeline);
  }
  const auto regs = machine.cpu().registers();
  constexpr unsigned sum = 21;
  constexpr unsigned fractional_shift = 24;
  require(machine.cpu().progress().instructions == instructions && regs.pc == boot_entry + 7 &&
              regs.x == 7 && regs.a == (std::uint64_t{sum} << fractional_shift) &&
              machine.bus().read({.space = DspSpace::x, .address = destination}) == 7,
          "Independent REP/ALU/store expectation failed after frontend restoration");
}

void all_cycle_cuts() {
  OwnDspMachine source(clocks);
  install(source);
  OwnDspTimeline timeline(source, timing, 0, reset);
  require(timeline.advance_reset_to(hold_ticks) == hold_ticks, "Lost held reset ticks");
  timeline.release_hardware_reset();
  require(timeline.advance_reset_to(recovery_end) == recovery_end - hold_ticks,
          "Lost reset recovery ticks");
  unsigned cuts = 0;
  while (source.cpu().progress().instructions < instructions) {
    const auto checkpoint = timeline.checkpoint();
    const auto deadline = timeline.next_deadline();
    const auto registers = source.cpu().registers();
    OwnDspMachine restored(clocks);
    OwnDspTimeline replay(restored, checkpoint);
    require(replay.next_deadline() == deadline && restored.cpu().registers() == registers &&
                restored.bus().last_dma_cycle() == source.bus().last_dma_cycle(),
            "Chip/frontend/DMA deadline was not restored at the exact cycle cut");
    finish(restored, replay);
    advance(source, timeline);
    ++cuts;
  }
  require(cuts > instructions, "Cuts did not include partial native/fetch cycles");
  finish(source, timeline);
  std::cout << "Restored frontend/native cycle cuts=" << cuts << '\n';
}

void held_reset_and_refusals() {
  OwnDspMachine source(clocks);
  install(source);
  OwnDspTimeline timeline(source, timing, 0, reset);
  constexpr unsigned partial = 23;
  require(timeline.advance_reset_to(partial) == partial, "Lost fractional EXTAL reset hold");
  const auto checkpoint = timeline.checkpoint();
  OwnDspMachine restored(clocks);
  OwnDspTimeline replay(restored, checkpoint);
  require(restored.clock().elapsed_ticks() == partial && restored.clock().hardware_reset_active(),
          "Held reset snapshot did not retain the partial EXTAL phase");
  for (auto *owner : {&timeline, &replay}) {
    require(owner->advance_reset_to(hold_ticks) == hold_ticks - partial, "Reset hold changed");
    owner->release_hardware_reset();
    require(owner->advance_reset_to(recovery_end) == recovery_end - hold_ticks,
            "Reset recovery changed");
  }
  require(replay.next_deadline() == timeline.next_deadline(), "Recovered clocks diverged");
  finish(source, timeline);
  finish(restored, replay);
  const auto stale = restored.checkpoint();
  restored.restore(stale);
  rejects([&] { static_cast<void>(replay.checkpoint()); });
  OwnDspMachine empty_target(clocks);
  rejects([&] { const OwnDspTimeline invalid(empty_target, OwnDspTimelineCheckpoint{}); });
  auto different_clock = clocks;
  different_clock.timebase_hz *= 2;
  OwnDspMachine different(different_clock);
  const auto before = different.cpu().registers();
  rejects([&] { const OwnDspTimeline invalid(different, checkpoint); });
  require(different.cpu().registers() == before && different.clock().elapsed_ticks() == 0,
          "Mismatched clocks changed the target before restore rejection");
}
}

int main() {
  try {
    all_cycle_cuts();
    held_reset_and_refusals();
    std::cout << "Own DSP frontend/timeline checkpoint controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
