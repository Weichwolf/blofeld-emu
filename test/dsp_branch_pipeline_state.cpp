// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_branch_pipeline_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::test::branch_pipeline {
namespace {
using native_access::fault;
constexpr unsigned unknown_target = 0x800;

void snapshots(const bool taken) {
  const std::array<std::string, 3> code{"jset #" + std::to_string(taken ? 2 : 3) + ",omr," +
                                            number(body + 4),
                                        "movep #>$654321,x:<<$fffffc",
                                        "move #>$654321,x0"};
  for (unsigned clock = 3; clock <= 10; ++clock) {
    DspMachine machine;
    prepare(machine, code);
    auto expected = machine.cpu().debug_state();
    expected.cpu.pc = body + 6;
    expected.cpu.x = (expected.cpu.x & ~word_mask) | literal;
    expected.native_instructions += taken ? 2U : 3U;
    expected.native_cycles += taken ? 6U : 8U;
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto at_checkpoint = machine.cpu().debug_state();
    const auto latch = machine.bus().once_latch();
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == at_checkpoint && machine.bus().once_latch() == latch,
              "Joint restore retains the real branch prefix, target samples and held Decode");
      clocks(run, machine, (taken ? 11U : 14U) - clock);
      require(machine.cpu().debug_state() == expected &&
                  machine.bus().once_latch() == (taken ? std::nullopt : std::optional(literal)) &&
                  run.completed_cycles() == (taken ? 11U : 14U),
              "Both branch continuations preserve actual native counts, flags, path I/O and time");
    }
  }
}

void fresh_same_address_target() {
  DspMachine machine;
  const std::array<std::string, 1> code{"jset #2,omr," + number(body)};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 4);
  machine.bus().write({.address = body, .value = 0});
  machine.bus().write({.address = body + 1, .value = 0});
  clocks(run, machine, 7);
  expected.cpu.pc = body + 1;
  expected.native_instructions += 2;
  expected.native_cycles += 5;
  require(machine.cpu().debug_state() == expected && run.completed_cycles() == 11,
          "The first branch consumes its old F2 pair; the same-address target uses fresh NOP data");
}

void failed_target_f2_and_recovery() {
  DspMachine machine;
  const std::array<std::string, 1> code{"jset #2,omr," + number(unknown_target)};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 4);
  const auto saved = run.checkpoint(machine);
  clocks(run, machine, 1);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("P:0x000800") && error.contains("pc=") && error.contains("omr=") &&
              run.faulted() && run.completed_cycles() == 5 &&
              machine.cpu().debug_state() == expected && machine.bus().last_dma_cycle() == 6,
          "Unknown actual target F2 fails before branch E1/retirement or elapsed success");
  run.restore(saved, machine);
  const std::array<std::uint32_t, 8> target{};
  checkpoint::ram(machine.bus(), unknown_target, target);
  clocks(run, machine, 7);
  expected.cpu.pc = unknown_target + 1;
  expected.native_instructions += 2;
  expected.native_cycles += 5;
  require(machine.cpu().debug_state() == expected && run.completed_cycles() == 11,
          "Trusted joint recovery reads explicitly supplied own target RAM without altering code");
}

void reset_pending_branch() {
  const std::array<std::string, 1> code{"jset #2,omr," + number(body)};
  for (const unsigned clock : {4U, 6U}) {
    DspMachine machine;
    prepare(machine, code);
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    machine.cpu().reset_spi_boot();
    const auto reset = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("coordinated") && !machine.reference_fetch().active &&
                !machine.reference_pending() && machine.cpu().debug_state() == reset &&
                run.completed_cycles() == clock,
            "Reset discards pending branch fetch/result state and refuses the stale clock owner");
    machine.cpu().step();
    auto expected = reset;
    expected.cpu.pc += 2;
    expected.cpu.r[0] = fetch_clock::initial_r0;
    ++expected.native_instructions;
    expected.native_cycles += 2;
    require(machine.cpu().debug_state() == expected,
            "After reset the actual own ROM entry executes, without a stale branch continuation");
  }
}

void unsupported_sources_stay_strict() {
  for (const auto *source : {"sr", "x0", "x:<<$ffff91"}) {
    DspMachine machine;
    const std::array<std::string, 1> code{"jset #2," + std::string(source) + ',' +
                                          number(body + 4)};
    prepare(machine, code);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 2);
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("automatic DSP decode P:") && error.contains("pc=") &&
                error.contains("omr=") && run.faulted() && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before && !machine.bus().once_latch(),
            "OMR branch support never admits SR/data/peripheral sources or their operand effects");
  }
}
}

void state_checks() {
  snapshots(false);
  snapshots(true);
  fresh_same_address_target();
  failed_target_f2_and_recovery();
  reset_pending_branch();
  unsupported_sources_stay_strict();
}
}
