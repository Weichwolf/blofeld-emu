// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu_dsp_reset_fixture.hpp"
#include "mcu_dsp_runtime_fixture.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test::mcu_reset {
void running_reset() {
  // LDAB1/STAB PORTU;NOP;NOP;LDAB0/STAB PORTU;LDAB1/STAB PORTU.
  // The three real write slots are480,840,1080; each STAB retains its trailing O.
  constexpr std::array<std::uint8_t, 17> program{0xc6,
                                                 1,
                                                 0x7b,
                                                 2,
                                                 0x78,
                                                 mcu_dsp::nop,
                                                 mcu_dsp::nop,
                                                 0xc6,
                                                 0,
                                                 0x7b,
                                                 2,
                                                 0x78,
                                                 0xc6,
                                                 1,
                                                 0x7b,
                                                 2,
                                                 0x78};
  auto contract = options(program);
  auto reset = mcu_dsp::reset_inputs();
  reset.recovery_cycles = 3;
  contract.runtime.initial_reset = reset;
  Fixture f(contract);
  configure(f);
  constexpr unsigned assertion = 840;
  constexpr unsigned second_release = 1080;
  constexpr unsigned recover = 135;
  constexpr unsigned initial_native = 5;
  constexpr unsigned initial_clk2 = 2;
  constexpr unsigned held_cut = 900;
  f.on_write = [&](const BusEvent &event) {
    const auto pins = f.mcu.bus().gpio_outputs(McuPort::u);
    require((pins.mask & pins.known & 1U) != 0 && (pins.levels & 1U) == event.value,
            "Reset signal observer did not see the genuine native PORTU output");
    if (event.value == 0) {
      const auto pending = f.dsp.cpu().pending_instruction();
      require(f.runtime->end_ticks() == assertion && pending &&
                  pending->effect.kind == dsp56300::EffectKind::retire &&
                  f.dsp.cpu().registers().pc == mcu_dsp::boot + 2 &&
                  f.dsp.cpu().progress().instructions == 1 &&
                  f.runtime->completed_cycles() == initial_native,
              "Native RESET assertion did not meet its real partial-PCU/retirement boundary");
      f.runtime->begin_dsp_reset(reset);
      require(!f.dsp.cpu().pending_instruction() && f.dsp.cpu().registers().pc == mcu_dsp::boot &&
                  f.dsp.cpu().progress() == dsp56300::Progress{} &&
                  f.dsp.clock().timer_clk2_edges() == initial_clk2,
              "Actual reset retired a cancelled instruction or discarded lifetime CLK2 time");
      observe_nops(f, second_release + recover, initial_native);
      return;
    }
    const auto at = f.runtime->end_ticks();
    require(event.value == 1 && (at == native_release || at == second_release),
            "Native reset release moved outside its independent MCU write slot");
    f.runtime->release_dsp_reset();
    observe_nops(f, at + recover, at == native_release ? 0 : initial_native);
  };
  f.run(6);
  held(f, held_cut, initial_native, initial_clk2);
  require(f.writes.size() == 2 && f.writes[0].value == 1 && f.writes[1].value == 0,
          "Actual MCU program lost its initial high-to-low reset sequence");
  const auto snapshot = f.runtime->checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(snapshot);
    held(f, held_cut, initial_native, initial_clk2);
    f.run(2);
    constexpr unsigned trailing_end = second_release + mcu_period;
    require(f.writes.size() == 1 && f.writes[0].value == 1 &&
                f.runtime->end_ticks() == trailing_end &&
                f.dsp.clock().hardware_reset_remaining_ticks() == recover - mcu_period,
            "Held native replay did not release reset before its real trailing O interval");
    f.run(10);
    constexpr unsigned reset_lifetime =
        native_release + recover + (second_release - assertion) + recover;
    running(f,
            {.until = trailing_end + (10 * mcu_period),
             .recovery_end = second_release + recover,
             .reset_ticks = reset_lifetime,
             .previous_cycles = initial_native,
             .previous_clk2 = initial_clk2});
    require(f.runtime->completed_cycles() == 16 && f.dsp.clock().timer_clk2_edges() == 7,
            "Reset kept an odd old CLK2 residue instead of starting the fresh divider epoch");
  }
}
}
