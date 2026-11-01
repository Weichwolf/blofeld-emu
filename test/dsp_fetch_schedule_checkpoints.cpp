// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_schedule_fixture.hpp"

#include <memory>

namespace blofeld::test::fetch_events {
void checkpoints() {
  constexpr unsigned initial_nops = 3;
  constexpr unsigned remaining_nops = 15;
  for (const auto order : {DspPeerOrder::peer_first, DspPeerOrder::dsp_first}) {
    Fixture source(order, {first_cycle, first_cycle + period, horizon});
    source.run(initial_nops);
    const auto snapshot = source.runtime->checkpoint();
    const auto cursor = source.source.cursor;
    source.source.log.clear();
    source.run(remaining_nops);
    const auto expected_log = source.source.log;
    const auto expected_mcu = detail::mcu_boundary(source.mcu);
    const auto expected_dsp = source.dsp.cpu().registers();
    for (unsigned replay = 0; replay < 2; ++replay) {
      source.restore(snapshot, cursor);
      source.run(remaining_nops);
      require(source.source.log == expected_log &&
                  detail::mcu_boundary(source.mcu) == expected_mcu &&
                  source.dsp.cpu().registers() == expected_dsp &&
                  source.runtime->completed_cycles() == cycles && source.source.cursor == 3,
              "Same-owner native timeline/source replay changed registers,clocks or input order");
    }
    Fixture clone(order);
    clone.source.identity = source.source.identity;
    clone.source.ticks = source.source.ticks;
    clone.restore(snapshot, cursor);
    clone.run(remaining_nops);
    require(clone.source.log == expected_log && detail::mcu_boundary(clone.mcu) == expected_mcu &&
                clone.dsp.cpu().registers() == expected_dsp &&
                clone.runtime->end_ticks() == horizon,
            "Fresh-owner checkpoint lost actual MCU/DSP state or source continuation");
    Fixture mismatch(order);
    mismatch.source.identity = source.source.identity;
    mismatch.source.ticks = {horizon + 1};
    mismatch.restore(snapshot, 0);
    const auto registers = mismatch.dsp.cpu().registers();
    const auto completed = mismatch.runtime->completed_cycles();
    require(rejects([&] { mismatch.step(); }).contains("pinned pending deadline") &&
                mismatch.dsp.cpu().registers() == registers &&
                mismatch.runtime->completed_cycles() == completed && mismatch.source.cursor == 0 &&
                mismatch.runtime->faulted(),
            "Clone executed work before validating its caller-restored pending source deadline");
  }
}
}
