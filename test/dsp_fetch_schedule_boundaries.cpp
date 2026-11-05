// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_schedule_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"

namespace blofeld::test::fetch_events {
void boundary_checks() {
  constexpr unsigned prefix = 3;
  constexpr unsigned suffix = 15;
  constexpr unsigned final_pc_offset = 16;
  constexpr unsigned mcu_reset_cycles = 5;
  constexpr unsigned mcu_end_ticks = 230;
  constexpr unsigned mcu_timebase = 60000000;
  for (const auto order : {DspPeerOrder::peer_first, DspPeerOrder::dsp_first}) {
    Fixture fixture(order, {first_cycle, first_cycle + period, horizon});
    auto expected_mcu = fixture.mcu.cpu().state();
    auto expected_dsp = fixture.dsp.cpu().registers();
    const auto mcu_matches = [&] {
      const auto actual = fixture.mcu.cpu().state();
      return actual.pc == expected_mcu.pc && actual.d == expected_mcu.d &&
             actual.x == expected_mcu.x && actual.y == expected_mcu.y &&
             actual.sp == expected_mcu.sp && actual.ccr == expected_mcu.ccr;
    };
    fixture.run(prefix);
    expected_mcu.pc += prefix;
    ++expected_dsp.pc;
    require(mcu_matches() && fixture.dsp.cpu().registers() == expected_dsp &&
                fixture.dsp.cpu().progress() == dsp56300::Progress{} &&
                fixture.runtime->completed_cycles() == 3 && fixture.source.cursor == 2,
            "Three MCU NOPs admit F1, F2 and native PC publication before DSP retirement");
    fixture.run(suffix);
    expected_mcu.pc += suffix;
    expected_dsp.pc = boot + final_pc_offset;
    require(mcu_matches() && fixture.dsp.cpu().registers() == expected_dsp &&
                fixture.dsp.cpu().progress() ==
                    dsp56300::Progress{.instructions = suffix, .estimated_cycles = suffix} &&
                fixture.runtime->completed_cycles() == cycles && fixture.source.cursor == 3 &&
                fixture.runtime->end_ticks() == horizon &&
                fixture.mcu.run_clock().timebase_hz() == mcu_timebase &&
                fixture.mcu.run_clock().elapsed_ticks() == mcu_end_ticks &&
                fixture.mcu.run_clock().bus_cycles() == mcu_reset_cycles + prefix + suffix &&
                fixture.dsp.clock().timer_clk2_edges() == final_pc_offset,
            "Eighteen MCU NOPs preserve all registers and admit fifteen DSP retirements by2070");
  }
}
}
