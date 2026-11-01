// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_reset_fixture.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test::fetch_reset {
namespace {
constexpr unsigned entry = 0x100, data = 0x400, canary = 0x654321;
constexpr unsigned high = 0x234567, low = 0x12345, long_store = 0x485c00;
constexpr unsigned bound = 256;

void prepare(Fixture &f) {
  // FM register selectors: R4, A0, A1, A2 and absolute JMP.
  constexpr std::array<std::uint32_t, 32> program{
      0x64f400, data, 0x50f400, low, 0x54f400, high, 0x52f400, 0, jump, entry};
  f.machine.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
  for (unsigned i = 0; i < 32; ++i) {
    f.machine.bus().write({.address = entry + i, .value = i == 0 ? long_store : 0});
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    f.machine.bus().write({.space = space, .address = data, .value = canary});
  }
  f.power_on();
}

bool pending_store(Fixture &f, const dsp56300::Space space) {
  const auto pending = f.machine.cpu().pending_instruction();
  return pending && pending->effect.kind == dsp56300::EffectKind::write &&
         pending->effect.pc == entry && pending->effect.address == data &&
         pending->effect.space == space;
}

bool retired_store(const Fixture &f) {
  const auto result = f.machine.cpu().instruction_outcome();
  return result && result->address == entry;
}

void select(Fixture &f, const unsigned cut) {
  for (unsigned i = 0; i < bound && !pending_store(f, dsp56300::Space::x); ++i) {
    static_cast<void>(f.step());
  }
  require(pending_store(f, dsp56300::Space::x) &&
              f.machine.cpu().registers().a == ((std::uint64_t{high} << accumulator_bits) | low),
          "Actual immediate instructions establish the literal48bit long-store source");
  if (cut > 0) {
    static_cast<void>(f.step());
    require(pending_store(f, dsp56300::Space::y),
            "The staged native long store retains its actual second data-bus write");
  }
  if (cut > 1) { static_cast<void>(f.step()); }
  if (cut > 2) {
    for (unsigned i = 0; i < bound && !retired_store(f); ++i) { static_cast<void>(f.step()); }
    require(retired_store(f), "The real long store reaches its actual retirement cut");
  }
}

void memory(Fixture &f, const unsigned x, const unsigned y) {
  require(f.machine.bus().read({.space = DspSpace::x, .address = data}) == x &&
              f.machine.bus().read({.space = DspSpace::y, .address = data}) == y,
          "Only actually accepted X/Y stores survive reset; pending writes never commit");
}

void store_case(const unsigned cut) {
  Fixture f(Start::blank);
  prepare(f);
  select(f, cut);
  const auto expected_x = cut > 0 ? high : canary;
  const auto expected_y = cut > 1 ? low : canary;
  memory(f, expected_x, expected_y);
  f.reset();
  memory(f, expected_x, expected_y);
  f.held(held_ticks, held_ticks);
  const auto saved = f.timeline->checkpoint();
  const auto ledger = f.ledger;
  f.machine.bus().write({.space = DspSpace::x, .address = data, .value = 7});
  Fixture fresh(Start::blank);
  fresh.restore(saved, ledger);
  memory(fresh, expected_x, expected_y);
  fresh.timeline->release_hardware_reset();
  fresh.held(recovery_ticks, recovery_ticks);
  memory(fresh, expected_x, expected_y);
  for (unsigned i = 0; i < bound && !retired_store(fresh); ++i) { static_cast<void>(fresh.step()); }
  memory(fresh, high, low);
  require(retired_store(fresh) && fresh.machine.cpu().registers().pc == entry + 1 &&
              fresh.machine.cpu().registers().r[4] == data + 1 &&
              fresh.machine.cpu().registers().a ==
                  ((std::uint64_t{high} << accumulator_bits) | low) &&
              fresh.machine.cpu().progress().instructions == 6,
          "Real reset entry repeats four MOVEs/JMP and the long store with literal pointer/data");
}
}

void store_checks() {
  // Explicit staged bus boundaries, not measured simultaneous silicon ports.
  for (unsigned cut = 0; cut < 4; ++cut) { store_case(cut); }
}
}
