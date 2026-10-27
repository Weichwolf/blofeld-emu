// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "fetch_clone_fixture.hpp"
#include "mcu/mcu_reset.hpp"

#include <exception>
#include <iostream>
#include <memory>

namespace {
using namespace clone_test;
constexpr unsigned continuation = 24, changed_mcu = 0x7e, changed_dsp = 0x654321;

void test_interleaved_forks() {
  for (unsigned cut = 0; cut < 10; ++cut) {
    DspMachine source;
    DspFetchRun live(dsp_inputs(), prepare(source));
    for (unsigned n = 0; n < cut; ++n) { independent(live.advance_cycle(source, pins()), source); }
    const auto saved = live.checkpoint(source);
    const auto original = frame(source, live);
    DspMachine target;
    DspFetchRun copied(saved, target);
    require(frame(source, live) == original && frame(target, copied) == original,
            "Fork construction preserves both real in-flight native states");
    for (unsigned n = 0; n < continuation; ++n) {
      const auto before = frame(source, live);
      independent(copied.advance_cycle(target, pins()), target);
      require(frame(source, live) == before,
              "Each copied pipeline period leaves the original pending frontend/operand state "
              "untouched");
      independent(live.advance_cycle(source, pins()), source);
      require(
          frame(source, live) == frame(target, copied),
          "Interleaved original and copied MOVE/CLR/ADD periods retain exact independent results");
    }
  }
}

void test_nonzero_event_floor() {
  Cursor cursor;
  Owners source;
  // Explicit late attachment: the actual native POR/vector sequence has already run.
  source.mcu.advance_system_reset(reset_edges, McuResetPad::high);
  auto late = inputs(false);
  late.epoch_ticks = vector_ticks;
  cursor.position = 3; // First supplied event is9300, after the literal9225 initial boundary.
  source.schedule =
      std::make_unique<McuDspFetchSchedule>(late, source.mcu, prepare(source.dsp), cursor.events());
  source.attach();
  require(source.schedule->end_ticks() == vector_ticks &&
              source.schedule->run().completed_cycles() == 0,
          "The late owner begins at the actual post-vector time without earlier DSP periods");
  const auto saved = source.schedule->checkpoint();
  const auto original = endpoint(source);
  const auto position = cursor.position;
  const auto queries = cursor.queries;
  cursor.position = 0;
  cursor.forbidden = true;
  Owners target;
  target.schedule =
      std::make_unique<McuDspFetchSchedule>(saved, target.mcu, target.dsp, cursor.events());
  require(cursor.queries == queries && equal(endpoint(source), original),
          "A stale cursor is not queried while staging a late snapshot");
  cursor.forbidden = false;
  target.attach();
  require(
      rejects([&] { target.step(); }).contains("predates its initial native MCU boundary") &&
          target.schedule->faulted() &&
          target.mcu.run_clock().elapsed_ticks() == source.mcu.run_clock().elapsed_ticks() &&
          equal(endpoint(source), original) && cursor.observed.empty(),
      "Transfer retains the exact nonzero initial event floor rather than replacing it with zero");
  cursor.position = position;
  source.step();
  require(!source.schedule->faulted() && source.schedule->run().completed_cycles() == 2,
          "Original late owners continue two genuine DSP periods after the discarded failed copy");
}

void test_memory_isolation() {
  Cursor cursor;
  Owners source;
  source.start(false, cursor);
  source.finish_reset();
  source.step();
  source.step();
  const auto saved = source.schedule->checkpoint();
  const auto original = endpoint(source);
  const auto queries = cursor.queries;
  cursor.forbidden = true;
  Owners target;
  target.schedule =
      std::make_unique<McuDspFetchSchedule>(saved, target.mcu, target.dsp, cursor.events());
  require(cursor.queries == queries && equal(endpoint(source), original),
          "Memory transfer stages without live input or native source mutation");
  // Explicit fixture writes distinguish independent RAM ownership from shallow aliases.
  target.mcu.bus().write(mcu_ram, changed_mcu);
  target.dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = changed_dsp});
  require(source.mcu.bus().read(mcu_ram) == marker &&
              target.mcu.bus().read(mcu_ram) == changed_mcu &&
              source.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word &&
              target.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == changed_dsp,
          "Distinct writes affect only copied MCU and DSP RAM");
  Owners second;
  second.schedule =
      std::make_unique<McuDspFetchSchedule>(saved, second.mcu, second.dsp, cursor.events());
  require(second.mcu.bus().read(mcu_ram) == marker &&
              second.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word &&
              equal(endpoint(source), original) && cursor.queries == queries,
          "The immutable checkpoint and original RAM survive another copy's writes");
}
}

int main() {
  try {
    test_interleaved_forks();
    test_nonzero_event_floor();
    test_memory_isolation();
    std::cout << "Containing clone:10interleaved24-period forks,nonzero event floor and MCU/DSP "
                 "RAM isolation passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
