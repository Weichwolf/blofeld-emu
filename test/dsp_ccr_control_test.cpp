// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_control_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned all_flags = 0xff;
constexpr unsigned alternating_flags = 0x55;
constexpr unsigned cleared_value_flags = 0x14;

void one_mask(const unsigned seed, const unsigned mask) {
  DspMachine machine;
  const std::array seeds{control::sr_move, masked_status | seed};
  const std::array body{0U, control::and_ccr | (mask << 8U), 0U};
  auto expected = control::prepare(machine, seeds, 1, body);
  require(expected.cpu.sr == (masked_status | seed),
          "Actual SR seed has the literal requested bits");
  DspFetchRun run(control::contract(), machine);
  unsigned retired = 0;
  unsigned held = 0;
  for (unsigned index = 0; index < 11; ++index) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if ((index == 4 || index == 5) && !phase.decode) { ++held; }
    if (!phase.retirement_published) { continue; }
    require(phase.execute2 && phase.execute2->address == boot + 2 + retired,
            "Only genuine own sequential words retire");
    const unsigned cost = retired == 1 ? 3U : 1U;
    ++expected.cpu.pc;
    ++expected.native_instructions;
    expected.native_cycles += cost;
    if (retired == 1) { expected.cpu.sr = masked_status | (seed & mask); }
    ++retired;
    control::check(machine, expected);
  }
  require(retired == 3 && held == 2,
          "ANDI has two actual Decode holds and a three-plus-one successor window");
}

void consecutive() {
  DspMachine machine;
  const std::array seeds{control::sr_move, masked_status | all_flags};
  const std::array body{0U, control::and_ccr | 0x5500U, control::and_ccr | 0xaa00U, 0U};
  auto expected = control::prepare(machine, seeds, 1, body);
  DspFetchRun run(control::contract(), machine);
  unsigned retired = 0;
  for (unsigned index = 0; index < 14; ++index) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if (!phase.retirement_published) { continue; }
    require(phase.execute2 && phase.execute2->address == boot + 2 + retired,
            "Consecutive CCR operations retain actual native order");
    ++expected.cpu.pc;
    ++expected.native_instructions;
    expected.native_cycles += retired == 1 || retired == 2 ? 3U : 1U;
    if (retired == 1) { expected.cpu.sr = masked_status | alternating_flags; }
    if (retired == 2) { expected.cpu.sr = masked_status; }
    ++retired;
    control::check(machine, expected);
  }
  require(retired == 4, "Two consecutive three-period CCR operations plus NOP take seven periods");
}

void restore_masks() {
  for (unsigned cut = 4; cut <= 10; ++cut) {
    DspMachine machine;
    const std::array seeds{control::sr_move, masked_status | all_flags};
    const std::array body{0U, control::and_ccr | 0x1400U, 0U};
    auto expected = control::prepare(machine, seeds, 1, body);
    expected.cpu.pc += 3;
    expected.cpu.sr = masked_status | cleared_value_flags;
    expected.native_instructions += 3;
    expected.native_cycles += 5;
    DspFetchRun run(control::contract(), machine);
    pipeline::clocks(run, machine, cut);
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      pipeline::clocks(run, machine, 11 - cut);
      control::check(machine, expected);
    }
  }
}
}

int main() {
  try {
    for (unsigned mask = 0; mask <= all_flags; ++mask) {
      one_mask(all_flags, mask);
      one_mask(alternating_flags, mask);
    }
    for (unsigned bit = 0; bit < 8; ++bit) {
      one_mask(1U << bit, all_flags);
      one_mask(all_flags, all_flags ^ (1U << bit));
    }
    consecutive();
    restore_masks();
    std::cout << "528 literal CCR masks, consecutive operations and14 restored suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
