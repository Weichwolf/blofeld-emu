// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_schedule_fixture.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace blofeld::test::fetch_events {
namespace {
void ordered(const DspPeerOrder order) {
  constexpr auto second = first_cycle + period;
  constexpr auto third = second + period;
  Fixture fixture(order,
                  {initial_ticks,
                   initial_ticks + 1,
                   first_cycle,
                   first_cycle + 1,
                   second,
                   second + 2,
                   third,
                   horizon});
  unsigned retired = 0;
  fixture.on_cycle = [&](const DspRuntimeCycle &cycle) {
    require(cycle.completion_ticks == reset_end + ((cycle.cycle + 1) * period) &&
                cycle.completion_ticks - cycle.start_ticks == period &&
                fixture.dsp.cpu().registers().a == 0 &&
                fixture.dsp.cpu().registers().sr == initial_status,
            "Actual native NOPs lost literal period, accumulator or reset flags");
    if (cycle.result.native && cycle.result.native->retired) { ++retired; }
  };
  fixture.run(mcu_instructions);
  std::vector<std::string> expected{"E450", "E451"};
  if (order == DspPeerOrder::peer_first) {
    expected.insert(expected.end(),
                    {"E630", "D630", "E631", "E675", "D675", "E677", "E720", "D720"});
  } else {
    expected.insert(expected.end(),
                    {"D630", "E630", "E631", "D675", "E675", "E677", "D720", "E720"});
  }
  require(fixture.source.log.size() == cycles + fixture.source.ticks.size() &&
              std::equal(expected.begin(), expected.end(), fixture.source.log.begin()) &&
              fixture.source.cursor == 8 && fixture.runtime->end_ticks() == horizon &&
              fixture.runtime->completed_cycles() == cycles &&
              fixture.dsp.clock().timer_clk2_edges() == 16 && retired == 15 &&
              fixture.dsp.cpu().progress().instructions == 15,
          "Literal event prefixes,33 native periods,15 NOP retirements and16 CLK2 edges differ");
  fixture.source.ticks.push_back(horizon + 1);
  fixture.step();
  require(fixture.source.cursor == 9 && fixture.runtime->completed_cycles() == cycles + 2,
          "A bound exhausted source could not append genuinely future input");
}

void reset_ties() {
  for (const auto order : {DspPeerOrder::peer_first, DspPeerOrder::dsp_first}) {
    Fixture fixture(order, {reset_end});
    fixture.source.on_observe = [&](std::uint64_t) {
      const bool before = order == DspPeerOrder::peer_first;
      require(fixture.dsp.clock().hardware_reset_active() == before &&
                  fixture.dsp.clock().elapsed_ticks() == reset_end - (before ? 1U : 0U) &&
                  fixture.runtime->completed_cycles() == 0 &&
                  fixture.dsp.clock().timer_clk2_edges() == 0,
              "Recovery edge tie lost the excluded single tick or fabricated native/CLK2 work");
    };
    fixture.step();
    const auto held = fixture.runtime->checkpoint();
    fixture.step();
    require(fixture.dsp.clock().elapsed_ticks() == first_cycle &&
                fixture.dsp.clock().hardware_reset_ticks() == reset_end &&
                fixture.runtime->completed_cycles() == 1 && fixture.source.cursor == 1,
            "Actual MCU periods did not complete135 reset ticks followed by45 native ticks");
    fixture.restore(held, 0);
    fixture.step();
    require(fixture.source.cursor == 1 && fixture.runtime->completed_cycles() == 1,
            "Held-reset replay lost its event source or genuine first native cycle");
  }
}

void strict_tie() {
  Fixture fixture(DspPeerOrder::reject_tie, {first_cycle});
  fixture.step();
  const auto registers = fixture.dsp.cpu().registers();
  require(rejects([&] { fixture.step(); }).contains("Unqualified simultaneous") &&
              fixture.runtime->faulted() && fixture.source.cursor == 0 &&
              fixture.runtime->completed_cycles() == 0 &&
              fixture.dsp.cpu().registers() == registers,
          "Unqualified native/input tie failed to retain its actual committed prefix");
}
}
}

int main() {
  try {
    using namespace blofeld;
    using namespace blofeld::test::fetch_events;
    guards();
    pin_capture();
    checkpoints();
    ordered(DspPeerOrder::peer_first);
    ordered(DspPeerOrder::dsp_first);
    reset_ties();
    strict_tie();
    std::cout << "Native MCU/DSP periods and external event boundaries passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
