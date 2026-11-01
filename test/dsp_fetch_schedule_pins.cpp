// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_schedule_fixture.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test::fetch_events {
void pin_capture() {
  constexpr unsigned pins = 0xfff;
  constexpr unsigned driven_word = 0xa5b;
  // DSP56371 register MOVEP: X:<<$ffffbd -> X0, then Y0 (register codes4/6).
  constexpr std::array<std::uint32_t, 2> code{0x0444dd, 0x0446dd};
  // F1,F2 then the actual read; register publication,PCU and retirement are
  // separate effects. The next MOVEP reads four reference cycles later.
  constexpr auto sample_tick = reset_end + (3 * period);
  constexpr auto second_sample = sample_tick + (4 * period);
  for (const auto order : {DspPeerOrder::peer_first, DspPeerOrder::dsp_first}) {
    Fixture fixture(order, {sample_tick, second_sample + 1}, code);
    auto expected = fixture.dsp.cpu().registers();
    expected.pc = boot + 3;
    expected.x = order == DspPeerOrder::peer_first ? driven_word : 0;
    expected.y = driven_word;
    auto &port = fixture.dsp.bus().gpio(DspPort::c);
    port.write({.reg = DspGpioRegister::control, .value = pins});
    port.set_drive({.mask = pins, .levels = 0});
    fixture.source.on_observe = [&](std::uint64_t) {
      port.set_drive({.mask = pins, .levels = fixture.source.cursor == 0 ? driven_word : 0});
    };
    fixture.on_cycle = [&](const DspRuntimeCycle &cycle) {
      if (cycle.completion_ticks == sample_tick) {
        require(fixture.dsp.cpu().registers().x == 0,
                "Actual GPIO read prematurely published its destination register");
      }
      if (cycle.completion_ticks == second_sample + period) {
        require(fixture.dsp.cpu().registers().y == driven_word &&
                    port.read(DspGpioRegister::data) == 0,
                "Native register publication reread the pins instead of its captured word");
      }
    };
    fixture.run(7);
    require(fixture.dsp.cpu().registers() == expected && fixture.source.cursor == 2 &&
                port.read(DspGpioRegister::data) == 0 &&
                fixture.dsp.cpu().progress().instructions == 2,
            "Native MOVEP did not retain tie-ordered samples across subsequent pin changes");
  }
}
}
