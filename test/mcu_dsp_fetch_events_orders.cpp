// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_dsp_fetch_events_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld::test::joint_events {
namespace {
constexpr auto d_first = McuDspTieOrder::dsp_before_mcu;
constexpr auto m_first = McuDspTieOrder::mcu_before_dsp;
constexpr auto e_first = McuExternalWriteOrder::external_before_mcu;
constexpr auto m_before_e = McuExternalWriteOrder::mcu_before_external;
constexpr auto ed = DspExternalOrder::external_before_dsp;
constexpr auto de = DspExternalOrder::dsp_before_external;
constexpr std::array cases{
    Case{
        .native = d_first, .external = e_first, .peers = ed, .sample = 2, .order = {'E', 'D', 'M'}},
    Case{
        .native = m_first, .external = e_first, .peers = ed, .sample = 3, .order = {'E', 'M', 'D'}},
    Case{
        .native = d_first, .external = e_first, .peers = de, .sample = 0, .order = {'D', 'E', 'M'}},
    Case{.native = d_first,
         .external = m_before_e,
         .peers = de,
         .sample = 0,
         .order = {'D', 'M', 'E'}},
    Case{.native = m_first,
         .external = m_before_e,
         .peers = ed,
         .sample = 3,
         .order = {'M', 'E', 'D'}},
    Case{.native = m_first,
         .external = m_before_e,
         .peers = de,
         .sample = 1,
         .order = {'M', 'D', 'E'}}};

void order_case(const Case policy) {
  Fixture f(policy);
  auto expected = f.dsp.cpu().debug_state();
  f.step(2);
  require(f.schedule.end_ticks() == collision && f.schedule.run().completed_cycles() == 4 &&
              f.cursor == 1 && f.levels == 3,
          "All three real actors complete the literal720tick boundary");
  std::vector<Event> tied;
  for (const auto event : f.trace) {
    if (event.time == collision) { tied.push_back(event); }
  }
  std::vector<Event> wanted;
  wanted.reserve(policy.order.size());
  for (const auto actor : policy.order) { wanted.push_back({.actor = actor, .time = collision}); }
  require(tied == wanted,
          "Literal three-actor order follows independently enumerated causal permutations");
  const auto checkpoint = f.schedule.checkpoint();
  expected.cpu.pc = memory_pipeline::body + 2;
  expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | policy.sample;
  expected.cpu.y = (expected.cpu.y & ~std::uint64_t{word_mask}) | 1;
  expected.native_instructions += 2;
  expected.native_cycles += 2;
  std::vector<Event> suffix;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.schedule.restore(checkpoint);
    f.cursor = 1;
    f.levels = 3;
    f.attach();
    const auto first = f.trace.size();
    f.step(2);
    const auto cpu = f.mcu.cpu().state();
    constexpr unsigned final_ccr = 0xd0;
    require(cpu.pc == entry + 6 && cpu.d == 1 && cpu.x == 0 && cpu.y == 0 && cpu.sp == byte_mask &&
                cpu.ccr == final_ccr && f.mcu.run_clock().bus_cycles() == 10 &&
                f.schedule.end_ticks() == finish && f.schedule.run().completed_cycles() == 8 &&
                f.schedule.run().elapsed_ticks() == 8 * period &&
                f.schedule.run().timer_clk2_edges() == 4 && f.dsp.cpu().debug_state() == expected &&
                f.cursor == 2 && f.levels == 1 &&
                f.dsp.bus().gpio(DspPort::c).read(DspGpioRegister::data) == 1 &&
                f.schedule.timeline_state().external_events == 2 &&
                f.mcu.bus().read(marker_address) == marker,
            "Both native CPUs,AG1/E2 captures,clock phases and peripheral state retain independent "
            "endpoints");
    const std::vector<Event> actual(f.trace.begin() + static_cast<std::ptrdiff_t>(first),
                                    f.trace.end());
    if (replay == 0) { suffix = actual; }
    require(actual == suffix,
            "Joint pipeline/MCU/external cursor restore retains the complete event suffix");
  }
}
}

void order_cases() {
  for (const auto policy : cases) { order_case(policy); }
}

void early_events() {
  constexpr std::uint64_t later_epoch = 900;
  constexpr std::uint64_t before_epoch = 500;
  constexpr std::uint64_t final_time = 990;
  Fixture f(cases.front(), later_epoch, {before_epoch, before_epoch + 1});
  const auto cpu = f.dsp.cpu().debug_state();
  f.step(1);
  require(
      f.cursor == 2 && f.schedule.timeline_state().external_events == 2 &&
          f.schedule.end_ticks() == epoch && f.schedule.run().completed_cycles() == 0 &&
          f.schedule.run().elapsed_ticks() == 0 && f.dsp.cpu().debug_state() == cpu,
      "Genuine early pin events survive without running the DSP before its declared future epoch");
  f.step(4);
  require(f.schedule.end_ticks() == final_time && f.schedule.run().completed_cycles() == 2 &&
              f.schedule.run().elapsed_ticks() == 2 * period && f.cursor == 2,
          "The first real native periods begin only at the future DSP epoch");
}
} // namespace blofeld::test::joint_events
