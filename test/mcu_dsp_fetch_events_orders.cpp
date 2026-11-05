// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_dsp_fetch_events_fixture.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace blofeld::test::joint_events {
namespace {
constexpr auto d_first = McuWriteOrder::before_mcu;
constexpr auto m_first = McuWriteOrder::after_mcu;
constexpr auto e_first = McuWriteOrder::before_mcu;
constexpr auto m_before_e = McuWriteOrder::after_mcu;
constexpr auto ed = DspPeerOrder::peer_first;
constexpr auto de = DspPeerOrder::dsp_first;
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
  auto expected = reset_registers();
  f.step(2);
  require(f.runtime->end_ticks() == collision && f.runtime->completed_cycles() == 3 &&
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
  expected.pc += 3;
  expected.x = policy.sample;
  expected.y = 1;

  f.step(4);
  const auto cpu = f.mcu.cpu().state();
  constexpr unsigned final_ccr = 0xd0;
  require(
      cpu.pc == entry + 8 && cpu.d == 1 && cpu.x == 0 && cpu.y == 0 && cpu.sp == byte_mask &&
          cpu.ccr == final_ccr && f.mcu.run_clock().bus_cycles() == 12 &&
          f.runtime->end_ticks() == finish &&
          f.runtime->completed_cycles() == (finish - recovery_end) / period &&
          f.dsp.clock().elapsed_ticks() == finish && f.dsp.clock().timer_clk2_edges() == 5 &&
          f.dsp.cpu().registers() == expected && f.cursor == 2 && f.levels == 1 &&
          f.dsp.bus().gpio(DspPort::c).read(DspGpioRegister::data) == 1 &&
          f.dsp.cpu().progress() == dsp56300::Progress{.instructions = 2, .estimated_cycles = 2} &&
          f.mcu.bus().read(marker_address) == marker,
      "Both native CPUs,accepted GPIO reads,clock phases and peripheral state retain independent "
      "endpoints");
}
}

void order_cases() {
  for (const auto policy : cases) { order_case(policy); }
}

void early_events() {
  constexpr std::uint64_t later_release = 900;
  constexpr std::uint64_t before_release = 500;
  constexpr std::uint64_t held_end = 540;
  constexpr std::uint64_t delayed_recovery = 1035;
  Fixture f(cases.front(), later_release, {before_release, before_release + 1});
  f.step(1);
  require(f.cursor == 2 && f.runtime->end_ticks() == held_end &&
              f.runtime->completed_cycles() == 0 && f.dsp.clock().elapsed_ticks() == held_end &&
              f.dsp.clock().hardware_reset_ticks() == held_end &&
              f.dsp.clock().hardware_reset_active() &&
              !f.dsp.clock().hardware_reset_remaining_ticks() &&
              f.dsp.cpu().registers() == reset_registers(),
          "Early pin events run during actual held DSP reset without native CPU execution");
  f.step(3);
  require(f.runtime->end_ticks() == later_release && f.runtime->completed_cycles() == 0,
          "Continued real MCU periods preserve delayed DSP reset hold");
  f.runtime->release_dsp_reset();
  f.step(2);
  require(f.runtime->end_ticks() == finish && f.runtime->completed_cycles() == 1 &&
              f.dsp.clock().elapsed_ticks() == finish &&
              f.dsp.clock().hardware_reset_ticks() == delayed_recovery &&
              !f.dsp.clock().hardware_reset_active() &&
              f.dsp.cpu().registers() == reset_registers() &&
              f.dsp.cpu().progress() == dsp56300::Progress{} && f.cursor == 2,
          "Literal delayed recovery precedes genuine F1; no epoch offset replaces reset time");
}
} // namespace blofeld::test::joint_events
