// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
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
    OwnDspMachine source(dsp_clock);
    prepare(source);
    OwnDspFetch live(source);
    for (unsigned n = 0; n < cut; ++n) { static_cast<void>(advance(live, source)); }
    const auto saved = live.checkpoint();
    const auto original = frame(source, &live);
    OwnDspMachine target(dsp_clock);
    OwnDspFetch copied(target, saved);
    require(frame(source, &live) == original && frame(target, &copied) == original,
            "Fork construction preserves both real in-flight native states");
    for (unsigned n = 0; n < continuation; ++n) {
      const auto before = frame(source, &live);
      const auto copied_cycle = advance(copied, target);
      require(frame(source, &live) == before,
              "Each copied period leaves the original pending CPU and fetch state untouched");
      const auto original_cycle = advance(live, source);
      require(
          copied_cycle == original_cycle && frame(source, &live) == frame(target, &copied),
          "Interleaved native periods retain complete state and independent MOVE/CLR/ADD results");
    }
  }
}

void test_nonzero_event_floor(const bool pending) {
  Cursor cursor;
  Owners source;
  source.mcu.advance_system_reset(reset_edges, McuResetPad::high);
  cursor.position = pending ? 3U : static_cast<unsigned>(cursor.ticks.size());
  prepare(source.dsp, true);
  source.schedule =
      std::make_unique<McuDspRuntime>(inputs(), source.mcu, source.dsp, cursor.events());
  source.schedule->release_dsp_reset();
  source.attach();
  require(source.schedule->end_ticks() == vector_ticks &&
              source.schedule->completed_cycles() == 0 &&
              source.dsp.clock().hardware_reset_ticks() == vector_ticks,
          "Late attachment retains the DSP's actual complete held-reset time from zero");
  const auto saved = source.schedule->checkpoint();
  const auto original = endpoint(source);
  const auto position = cursor.position;
  const auto queries = cursor.queries;
  cursor.position = 0;
  cursor.forbidden = true;
  Owners target;
  target.schedule = std::make_unique<McuDspRuntime>(saved, target.mcu, target.dsp, cursor.events());
  require(cursor.queries == queries && equal(endpoint(source), original),
          "A stale cursor is not queried while staging a nonzero-boundary snapshot");
  cursor.forbidden = false;
  target.attach();
  require(
      rejects([&] { target.step(); })
              .contains(pending ? "pinned pending deadline"
                                : "moved backwards into a consumed timeline") &&
          target.schedule->faulted() &&
          target.mcu.run_clock().elapsed_ticks() == source.mcu.run_clock().elapsed_ticks() &&
          equal(endpoint(source), original) && cursor.observed.empty(),
      "A copy retains its pending deadline or closed event floor without consuming stale input");
  cursor.position = position;
  source.step();
  require(!source.schedule->faulted() && source.schedule->completed_cycles() == 0 &&
              source.dsp.clock().hardware_reset_ticks() == vector_ticks + (period * 2),
          "Actual first MCU period completes DSP recovery without fabricated native work");
  source.step();
  require(!source.schedule->faulted() && source.schedule->completed_cycles() == 6,
          "Actual three-period MCU store admits exactly six subsequent DSP periods");
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
  target.schedule = std::make_unique<McuDspRuntime>(saved, target.mcu, target.dsp, cursor.events());
  require(cursor.queries == queries && equal(endpoint(source), original),
          "Memory transfer stages without live input or native source mutation");
  target.mcu.bus().write(mcu_ram, changed_mcu);
  target.dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = changed_dsp});
  require(source.mcu.bus().read(mcu_ram) == marker &&
              target.mcu.bus().read(mcu_ram) == changed_mcu &&
              source.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word &&
              target.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == changed_dsp,
          "Distinct writes affect only copied MCU and DSP RAM");
  Owners second;
  second.schedule = std::make_unique<McuDspRuntime>(saved, second.mcu, second.dsp, cursor.events());
  require(second.mcu.bus().read(mcu_ram) == marker &&
              second.dsp.bus().read({.space = DspSpace::x, .address = dsp_ram}) == dsp_word &&
              equal(endpoint(source), original) && cursor.queries == queries,
          "The immutable checkpoint and original RAM survive another copy's writes");
}
}

int main() {
  try {
    test_interleaved_forks();
    test_nonzero_event_floor(false);
    test_nonzero_event_floor(true);
    test_memory_isolation();
    std::cout << "Clone isolation:10interleaved24-period forks,two nonzero event-floor profiles "
                 "and deep MCU/DSP RAM isolation passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
