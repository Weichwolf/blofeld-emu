// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_machine.hpp"
#include "dsp_fetch_native_fixture.hpp"
#include "dsp_native_loop_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace blofeld::test::native_loop {
namespace {
using fixture::require;
constexpr unsigned count = 3, width = 4, total_periods = 17, instructions = 13;

std::array<std::uint32_t, padding_words> program() {
  std::array<std::uint32_t, padding_words> result{};
  result[0] = dor_three; // Authored DOR #3 over four NOPs.
  result[1] = 5;
  return result;
}

void suffix(fixture::Fixture &f, const std::uint64_t target) {
  constexpr unsigned bound = 64;
  for (unsigned index = 0; f.machine.cpu().progress().instructions < target && index < bound;
       ++index) {
    static_cast<void>(advance(f));
  }
  require(f.machine.cpu().progress().instructions == target,
          "The actual native suffix reaches its independent instruction bound");
}

void observed_boundaries() {
  for (const unsigned cut : {1U, 4U, 5U, 8U, 9U, 10U, 13U, total_periods}) {
    fixture::Fixture f(program());
    attach(f);
    const auto initial = f.machine.cpu().registers();
    const auto progress = f.machine.cpu().progress();
    const auto ticks = f.machine.clock().elapsed_ticks();
    for (unsigned elapsed = 0; elapsed < cut; ++elapsed) { static_cast<void>(advance(f)); }
    auto prefix = opened(initial, {.count = count, .width = width});
    const auto completed = cut < setup_periods ? 0U : cut - setup_periods;
    if (completed == count * width) {
      prefix = finished(initial, {.count = count, .width = width});
    } else {
      prefix.pc = body + completed % width;
      prefix.lc = count - completed / width;
    }
    require(f.machine.cpu().registers() == prefix,
            "Each observed boundary has independently expected full loop and stack state");
    const auto pending = f.machine.cpu().pending_instruction();

    if (pending) {
      fixture::Fixture foreign(program());
      attach(foreign);
      for (unsigned elapsed = 0; elapsed < cut; ++elapsed) { static_cast<void>(advance(foreign)); }
      require(!fixture::rejects([&] {
                 static_cast<void>(foreign.machine.advance(pending->ticket));
               }).empty(),
              "An independently reset native owner rejects the original source effect ticket");
      require(foreign.machine.cpu().registers() == prefix,
              "The foreign-ticket refusal leaves the independently expected foreign state intact");
    }
    require(f.machine.cpu().registers() == prefix,
            "A refused old ticket leaves the literal original prefix unchanged");
    suffix(f, progress.instructions + instructions);
    require(f.machine.cpu().registers() == finished(initial, {.count = count, .width = width}) &&
                f.machine.cpu().progress().estimated_cycles ==
                    progress.estimated_cycles + total_periods &&
                f.machine.clock().elapsed_ticks() == ticks + total_periods * period,
            "The real continuation retains full native results and zero-overhead timing");
  }
}

void changed_program() {
  fixture::Fixture f(program());
  attach(f);
  const auto initial = f.machine.cpu().registers();
  const auto progress = f.machine.cpu().progress();
  for (unsigned elapsed = 0; elapsed < setup_periods; ++elapsed) { static_cast<void>(advance(f)); }
  constexpr unsigned short_move_r0 = 0x303400;
  constexpr unsigned changed_r0 = 0x34;
  f.machine.bus().write({.address = body, .value = short_move_r0});
  static_cast<void>(advance(f));
  require(f.machine.cpu().registers().r[0] == fixture::initial_r0,
          "A captured first-iteration NOP remains owned after external program memory changes");
  for (unsigned elapsed = 0; elapsed < width; ++elapsed) { static_cast<void>(advance(f)); }
  require(f.machine.cpu().registers().r[0] == changed_r0,
          "A genuinely new second-iteration F2 read observes the changed MOVE opcode");
  suffix(f, progress.instructions + instructions);
  auto expected = finished(initial, {.count = count, .width = width});
  expected.r[0] = changed_r0;
  require(f.machine.cpu().registers() == expected && f.machine.cpu().progress().estimated_cycles ==
                                                         progress.estimated_cycles + total_periods,
          "Loop prediction preserves actual F2 values instead of caching an old body");
}

void unknown_body() {
  constexpr std::array<std::uint32_t, 2> code{dor_three, 5};
  fixture::Fixture f(code);
  attach(f);
  const auto progress = f.machine.cpu().progress();
  const auto ticks = f.machine.clock().elapsed_ticks();
  for (unsigned elapsed = 0; elapsed < setup_periods; ++elapsed) { static_cast<void>(advance(f)); }
  const auto message = fixture::rejects([&] { static_cast<void>(advance(f)); });
  require(message.contains("Unknown DSP program sample read P:0x000102") &&
              f.machine.cpu().progress().instructions == progress.instructions + 1 &&
              f.machine.clock().elapsed_ticks() == ticks + setup_periods * period,
          "An unknown loop-body F2 fails at native use with its address and no fabricated step");
}

void reset_opening() {
  fixture::Fixture f(program());
  attach(f);
  static_cast<void>(advance(f));
  require((f.machine.cpu().registers().sr & loop_flag) != 0,
          "A real pending DO has already published its active native loop frame");
  f.machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(!fixture::rejects([&] { static_cast<void>(f.fetch->view()); }).empty(),
          "Hardware reset invalidates the captured owner of an unretired loop opening");
  constexpr auto reset_ticks = 2 * period;
  require(f.machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "The actual second reset observes its complete asserted hold");
  f.machine.release_hardware_reset();
  require(f.machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "The actual second reset completes recovery before new opcode admission");
  f.fetch.reset();
  f.attach();
  require(f.machine.cpu().progress() == dsp56300::Progress{} &&
              (f.machine.cpu().registers().sr & loop_flag) == 0 &&
              f.machine.cpu().registers().sp == 0 && f.machine.cpu().registers().sc == 0,
          "A fresh reset owner starts without a prior loop frame or instruction budget");
  suffix(f, 1);
  constexpr unsigned boot = 0xff0000;
  require(f.machine.cpu().registers().pc == boot + 2 &&
              f.machine.cpu().registers().r[0] == fixture::initial_r0 &&
              f.machine.cpu().progress().estimated_cycles == 2,
          "The new owner executes the real authored two-word R0 bootstrap after reset");
}
} // namespace

void state_checks() {
  observed_boundaries();
  changed_program();
  unknown_body();
  reset_opening();
  std::cout
      << "Loop boundaries, foreign tickets, changed P-memory, unknown F2 and reset checks passed\n";
}
} // namespace blofeld::test::native_loop
