// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_fetch_schedule_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::fetch_events {
void pin_capture() {
  constexpr unsigned pins = 0xfff;
  constexpr unsigned driven_word = 0xa5b;
  constexpr auto sample_tick = 4 * period;
  const std::array<std::string, 2> code{"movep x:<<$ffffbd,x0", "movep x:<<$ffffbd,y0"};
  for (const auto order :
       {DspExternalOrder::external_before_dsp, DspExternalOrder::dsp_before_external}) {
    DspMachine machine;
    memory_pipeline::prepare(machine, code);
    auto &port = machine.bus().gpio(DspPort::c);
    port.write({.reg = DspGpioRegister::control, .value = pins});
    port.set_drive({.mask = pins, .levels = 0});
    auto expected = machine.cpu().debug_state();
    DspFetchScheduledRun schedule(inputs(), machine);
    Source source{.ticks = {sample_tick, (5 * period) + 1}, .order = order};
    auto events = source.events();
    events.observe = [&](const std::uint64_t tick) {
      require(source.ticks.at(source.cursor) == tick, "Actual pin event cursor matches its time");
      port.set_drive({.mask = pins, .levels = source.cursor == 0 ? driven_word : 0});
      ++source.cursor;
    };
    require(
        schedule.advance_to(
            {.horizon = {.ticks = 8 * period}, .maximum_cycles = 8}, cycle_inputs, {}, events) == 8,
        "Both native peripheral readers execute through real pipeline periods");
    expected.cpu.pc = memory_pipeline::body + 2;
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) |
                     (order == DspExternalOrder::external_before_dsp ? driven_word : 0);
    expected.cpu.y = (expected.cpu.y & ~std::uint64_t{word_mask}) | driven_word;
    expected.native_instructions += 2;
    expected.native_cycles += 2;
    require(machine.cpu().debug_state() == expected && source.cursor == 2 &&
                schedule.state().external_events == 2 && port.read(DspGpioRegister::data) == 0,
            "Native AG1 samples exact tie-ordered GPIO inputs; later pin changes cannot rewrite "
            "captured values before actual E2 publication");
  }
}
} // namespace blofeld::test::fetch_events
