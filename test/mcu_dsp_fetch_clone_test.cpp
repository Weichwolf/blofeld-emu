// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "fetch_clone_fixture.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>

namespace {
using namespace clone_test;
enum class Cut : std::uint8_t { zero, settling, recovery, running };

void prefix(Owners &owner, const Cut cut) {
  if (cut == Cut::zero) { return; }
  if (cut == Cut::running) {
    owner.finish_reset();
    owner.step();
    owner.step();
    return;
  }
  owner.mcu.advance_system_reset(settle_edge, McuResetPad::unknown);
  if (cut == Cut::recovery) {
    owner.schedule->release_dsp_reset();
    owner.mcu.advance_system_reset(1, McuResetPad::unknown);
    require(owner.schedule->run().hardware_reset_remaining_ticks() == period,
            "The cut retains one literal EXTAL period before native DSP recovery");
  }
}

void suffix(Owners &owner) {
  owner.finish_reset();
  if (owner.mcu.cpu().state().pc == entry) { owner.step(); }
  if (owner.mcu.cpu().state().pc == entry + 2) { owner.step(); }
  for (unsigned n = 0; n < 4; ++n) { owner.step(); }
  constexpr std::uint64_t mcu_end = 1105;
  constexpr unsigned common_factor = 9;
  const auto ticks = mcu_end - owner.initial_phase;
  require(owner.mcu.cpu().state().pc == entry + 9 && owner.mcu.run_clock().bus_cycles() == 13 &&
              owner.mcu.run_clock().reset_sysclk_edges() == reset_edges &&
              owner.mcu.run_clock().elapsed_ticks() == ticks &&
              owner.schedule->end_ticks() == ticks * common_factor &&
              owner.schedule->run().completed_cycles() ==
                  (owner.schedule->end_ticks() - owner.schedule->run().hardware_reset_ticks()) /
                      period,
          "Literal195SYSCLK+five-vector+1+3+four-NOP periods retain independent full clock totals");
  require(owner.mcu.bus().read(mcu_ram) == marker && (owner.mcu.cpu().state().d >> 8U) == marker &&
              owner.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word,
          "Own native LDAA/STAA produces literal SRAM data; DSP fixture RAM survives transfer");
  require(rejects([&] {
            static_cast<void>(owner.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram + 1}));
          }).contains("Unknown DSP memory read"),
          "Unknown adjacent DSP RAM stays unknown");
}

void test_suffix(const Cut cut, const bool held, const unsigned phase) {
  Cursor cursor;
  Owners source(phase);
  source.start(held, cursor);
  prefix(source, cut);
  const auto at_cut = endpoint(source);
  const auto snapshot = source.schedule->checkpoint();
  const auto saved_position = cursor.position;
  const auto saved_events = cursor.observed;
  source.trace.clear();
  suffix(source);
  const auto expected = endpoint(source);
  const auto trace = source.trace;
  const auto events = cursor.observed;
  for (unsigned replay = 0; replay < 2; ++replay) {
    cursor.position = saved_position;
    cursor.observed = saved_events;
    const auto queries = cursor.queries;
    cursor.forbidden = true;
    Owners target(phase);
    target.schedule =
        std::make_unique<McuDspFetchSchedule>(snapshot, target.mcu, target.dsp, cursor.events());
    require(cursor.queries == queries && cursor.observed == saved_events &&
                equal(endpoint(target), at_cut) && equal(endpoint(source), expected),
            "Fresh staged owners reproduce every native/clock state without calling live input");
    const auto own = target.schedule->checkpoint();
    require(rejects([&] { target.schedule->restore(snapshot); }).contains("matching inactive") &&
                !target.schedule->faulted(),
            "Explicit clone never turns a foreign snapshot into a same-owner restore token");
    target.schedule->restore(own);
    require(cursor.queries == queries && equal(endpoint(target), at_cut),
            "The clone's own checkpoint restores with the retained event floor and no callback");
    cursor.forbidden = false;
    target.attach();
    suffix(target);
    require(equal(endpoint(target), expected) && target.trace == trace &&
                cursor.observed == events && equal(endpoint(source), expected),
            "Actual post-clone CPU/pipeline/operand traces and every external event match the "
            "source suffix");
  }
}

void test_rejections() {
  Cursor cursor;
  Owners source;
  source.start(false, cursor);
  source.finish_reset();
  const auto snapshot = source.schedule->checkpoint();
  const auto before = endpoint(source);
  const auto saved_position = cursor.position;
  const auto queries = cursor.queries;
  cursor.forbidden = true;
  for (unsigned fault = 0; fault < 7; ++fault) {
    Owners target;
    const auto initial = target.mcu.run_clock().elapsed_ticks();
    auto events = cursor.events();
    switch (fault) {
      case 0: events.identity = std::make_shared<DspFetchEventIdentity>(); break;
      case 1: events.order = DspExternalOrder::dsp_before_external; break;
      case 2: ++events.maximum_events; break;
      case 3: events.next_ticks = {}; break;
      case 4: events.observe = {}; break;
      case 5: target.mcu.advance_system_reset(1, McuResetPad::unknown); break;
      case 6: prepare(target.dsp); break;
      default: throw Fault("Invalid fixed admission case");
    }
    const auto actual_ticks = target.mcu.run_clock().elapsed_ticks();
    require(!rejects([&] {
               const McuDspFetchSchedule rejected(snapshot, target.mcu, target.dsp, events);
             }).empty() &&
                target.mcu.run_clock().elapsed_ticks() == actual_ticks &&
                cursor.queries == queries && cursor.position == saved_position &&
                equal(endpoint(source), before),
            "Malformed identity/order/budget/callbacks and non-pristine targets refuse before "
            "model input");
    require(fault == 5 || actual_ticks == initial,
            "Only the explicit non-fresh case advances its own clock");
  }
  Owners target;
  require(rejects([&] {
            const McuDspFetchSchedule rejected(snapshot, source.mcu, target.dsp, cursor.events());
          }).contains("distinct isolated"),
          "Live source MCU cannot be used as a clone destination");
  require(rejects([&] {
            const McuDspFetchSchedule rejected(snapshot, target.mcu, source.dsp, cursor.events());
          }).contains("distinct isolated"),
          "Live source DSP cannot be used as a clone destination");
  try {
    Owners staged;
    staged.schedule =
        std::make_unique<McuDspFetchSchedule>(snapshot, staged.mcu, staged.dsp, cursor.events());
    throw Fault("Own deterministic staged wiring refusal before publication");
  } catch (const Fault &) {
    require(equal(endpoint(source), before) && cursor.queries == queries,
            "Discarding a fully staged replacement leaves the original native owners and input "
            "untouched");
  }
}

void test_deferred_cursor_validation() {
  Cursor cursor;
  Owners source;
  source.start(false, cursor);
  source.finish_reset();
  const auto saved = source.schedule->checkpoint();
  const auto original = endpoint(source);
  const auto position = cursor.position;
  const auto events = cursor.observed;
  const auto queries = cursor.queries;
  cursor.position = 0; // Deliberately stale external cursor, still not observed during staging.
  cursor.forbidden = true;
  Owners target;
  target.schedule =
      std::make_unique<McuDspFetchSchedule>(saved, target.mcu, target.dsp, cursor.events());
  require(cursor.queries == queries && cursor.observed == events,
          "Stage construction never consults a currently stale external cursor");
  cursor.forbidden = false;
  target.attach();
  require(rejects([&] { target.step(); }).contains("external timestamp precedes") &&
              target.schedule->faulted() &&
              target.mcu.run_clock().elapsed_ticks() == source.mcu.run_clock().elapsed_ticks() &&
              equal(endpoint(source), original) && cursor.observed == events,
          "First real advancement rejects the stale cursor, quarantines the copy and retains "
          "source time");
  cursor.position = position;
  source.step();
  require(!source.schedule->faulted(),
          "The original owners continue after the discarded failed stage");
}

void test_expired_source_address_reuse() {
  std::optional<McuMachine> mcu;
  std::optional<DspMachine> dsp;
  std::optional<McuDspFetchSchedule> schedule;
  mcu.emplace(image(), BLOFELD_DEFAULT_SLA, contract(), McuPowerOnInputs{3});
  dsp.emplace();
  schedule.emplace(inputs(true), *mcu, *dsp);
  mcu->set_reset_period_observer(
      [&](const auto edge) { schedule->begin_reset_period(edge, pins); });
  mcu->advance_system_reset(13, McuResetPad::unknown);
  const auto saved = schedule->checkpoint();
  const auto *const old_mcu = std::addressof(*mcu);
  const auto *const old_dsp = std::addressof(*dsp);
  schedule.reset();
  mcu.reset();
  dsp.reset();
  mcu.emplace(image(), BLOFELD_DEFAULT_SLA, contract(), McuPowerOnInputs{3});
  dsp.emplace();
  require(std::addressof(*mcu) == old_mcu && std::addressof(*dsp) == old_dsp,
          "The controlled optional storage reuses both exact native object addresses");
  schedule.emplace(saved, *mcu, *dsp);
  require(schedule->end_ticks() == 13 * period && mcu->run_clock().reset_sysclk_edges() == 13 &&
              rejects([&] { schedule->restore(saved); }).contains("matching inactive") &&
              !schedule->faulted(),
          "Expired lifetime permits explicit transfer but rejects accidental same-address restore");
  const auto own = schedule->checkpoint();
  schedule->restore(own);
  mcu->set_reset_period_observer(
      [&](const auto edge) { schedule->begin_reset_period(edge, pins); });
  mcu->advance_system_reset(1, McuResetPad::unknown);
  require(schedule->end_ticks() == 14 * period,
          "New live owner advances the transferred real reset phase");
}
}

int main() {
  try {
    test_suffix(Cut::zero, false, 0);
    test_suffix(Cut::settling, false, 0);
    test_suffix(Cut::settling, false, 4);
    test_suffix(Cut::zero, true, 0);
    test_suffix(Cut::settling, true, 4);
    test_suffix(Cut::recovery, true, 0);
    test_suffix(Cut::running, false, 0);
    test_suffix(Cut::running, false, 4);
    test_rejections();
    test_expired_source_address_reuse();
    test_deferred_cursor_validation();
    std::cout << "Joint clone:8native profiles,16suffixes,9admission refusals and expired-address "
                 "transfer passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
