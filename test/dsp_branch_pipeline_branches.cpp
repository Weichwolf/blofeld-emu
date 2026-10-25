// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_branch_pipeline_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::branch_pipeline {
namespace {
void independent_encodings() {
  struct Reference {
    const char *assembly;
    unsigned opcode;
    unsigned target;
  };

  // Exact own pairs from NXP6.3.28, same fixed program used for the six
  // independently checked normal-mode branch/false-Decode timing windows.
  constexpr std::array references{
      Reference{.assembly = "jset #3,omr,$11a", .opcode = 0x0afa23, .target = 0x11a},
      Reference{.assembly = "jclr #3,omr,$130", .opcode = 0x0afa03, .target = 0x130},
      Reference{.assembly = "jset #2,omr,$146", .opcode = 0x0afa22, .target = 0x146},
      Reference{.assembly = "jclr #2,omr,$15c", .opcode = 0x0afa02, .target = 0x15c},
      Reference{.assembly = "jset #2,omr,$170", .opcode = 0x0afa22, .target = 0x170},
      Reference{.assembly = "jclr #2,omr,$184", .opcode = 0x0afa02, .target = 0x184}};
  for (const auto &reference : references) {
    require(std::ranges::equal(checkpoint::assemble(reference.assembly),
                               std::array{reference.opcode, reference.target}),
            "Own register branch opcode/extension must match the independent assembler");
  }
}

struct BranchCase {
  const char *operation;
  unsigned bit;
  bool taken;
};

void actual_branch(const BranchCase sample) {
  DspMachine machine;
  const std::array<std::string, 3> code{std::string(sample.operation) + " #" +
                                            std::to_string(sample.bit) + ",omr," + number(body + 4),
                                        "movep #>$654321,x:<<$fffffc",
                                        "move #>$654321,x0"};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.omr == spi_omr, "Actual SPI entry supplies the independent OMR bit inputs");
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 7);
  auto expected = before;
  expected.cpu.pc = sample.taken ? body + 4 : body + 2;
  ++expected.native_instructions;
  expected.native_cycles += 4;
  require(machine.cpu().debug_state() == expected && !machine.bus().once_latch(),
          "The first E2 publishes only the actual native branch with intact registers and flags");
  clocks(run, machine, sample.taken ? 4 : 7);
  expected.cpu.pc = body + 6;
  expected.cpu.x = (expected.cpu.x & ~word_mask) | literal;
  expected.native_instructions += sample.taken ? 1U : 2U;
  expected.native_cycles += sample.taken ? 2U : 4U;
  require(machine.cpu().debug_state() == expected &&
              machine.bus().once_latch() ==
                  (sample.taken ? std::nullopt : std::optional(literal)) &&
              run.completed_cycles() == (sample.taken ? 11U : 14U),
          "Four-clock taken and five-clock false issue spacing preserve real path I/O and results");
}

void discarded_path_has_no_effects() {
  DspMachine machine;
  const std::array<std::string, 5> code{"jset #2,omr," + number(body + 6),
                                        "move x:(r0)+,x0",
                                        "movep #>$654321,x:<<$fffffc",
                                        "stop",
                                        "move #>$654321,x0"};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 11);
  expected.cpu.pc = body + 8;
  expected.cpu.x = (expected.cpu.x & ~word_mask) | literal;
  expected.native_instructions += 2;
  expected.native_cycles += 6;
  require(machine.cpu().debug_state() == expected && !machine.bus().once_latch(),
          "Discarded AGU load, device write and unsupported opcode produce no data or R effect");
}

void sequential_target(const bool taken) {
  DspMachine machine;
  const std::array<std::string, 2> code{
      std::string(taken ? "jset" : "jclr") + " #2,omr," + number(body + 2), "move #>$654321,x0"};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, taken ? 11 : 12);
  expected.cpu.pc = body + 4;
  expected.cpu.x = (expected.cpu.x & ~word_mask) | literal;
  expected.native_instructions += 2;
  expected.native_cycles += 6;
  require(machine.cpu().debug_state() == expected && run.completed_cycles() == (taken ? 11U : 12U),
          "Equal sequential targets retain the genuine branch decision and extra false Decode");
}

void repeated_self_branch() {
  DspMachine machine;
  const std::array<std::string, 1> code{"jset #2,omr," + number(body)};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 7);
  for (unsigned iteration = 0; iteration < 3; ++iteration) {
    if (iteration != 0) { clocks(run, machine, 4); }
    ++expected.native_instructions;
    expected.native_cycles += 4;
    require(machine.cpu().debug_state() == expected && run.completed_cycles() == 7 + 4 * iteration,
            "Each self branch consumes a fresh owned F2 pair and retires once every four clocks");
  }
}

struct WindowCase {
  const char *operation;
  unsigned bit;
  bool taken;
  bool sequential;
  unsigned expected_clocks;
};

std::optional<unsigned> retired_pc(const DspFetchCycleResult &cycle) {
  if (!cycle.pipeline) { throw Fault("Expected the actual automatic pipeline result"); }
  const auto stages = *cycle.pipeline;
  if (!stages.retirement_published) { return std::nullopt; }
  if (!stages.execute2) { throw Fault("A real retirement lost its actual E2 word"); }
  return stages.execute2->address;
}

void independent_window(const WindowCase sample) {
  constexpr unsigned quiet = 8;
  constexpr unsigned maximum_clocks = 64;
  constexpr unsigned base_native_cycles = 20;
  constexpr unsigned x_canary = 0x13579b;
  constexpr unsigned y_canary = 0x2468ac;
  constexpr unsigned data = 0x100;
  const unsigned branch_pc = body + 2 + quiet;
  const unsigned target = branch_pc + 2 + (sample.sequential ? 0U : 2U);
  std::vector<std::string> code{"move #>0,x0"};
  code.insert(code.end(), quiet, "nop");
  code.push_back(std::string(sample.operation) + " #" + std::to_string(sample.bit) + ",omr," +
                 number(target));
  if (!sample.sequential) { code.emplace_back("move #>$654321,x0"); }
  code.insert(code.end(), quiet, "nop");
  DspMachine machine;
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  machine.bus().write({.space = DspSpace::x, .address = data, .value = x_canary});
  machine.bus().write({.space = DspSpace::y, .address = data, .value = y_canary});
  DspFetchRun run(contract(), machine);
  std::optional<std::uint64_t> before;
  std::optional<std::uint64_t> after;
  for (unsigned clock = 0; clock < maximum_clocks && !after; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto pc = retired_pc(cycle);
    if (!pc) { continue; }
    if (*pc == branch_pc - 1) { before = cycle.completed_cycles; }
    if (*pc == target + 5) { after = cycle.completed_cycles; }
  }
  if (!before || !after) { throw Fault("Actual retirement never reached both quiet anchors"); }
  const auto observed = *after - *before;
  require(observed == sample.expected_clocks,
          std::string(sample.operation) + (sample.taken ? " taken" : " false") +
              " primary/NXP quiet window expected=" + std::to_string(sample.expected_clocks) +
              " observed=" + std::to_string(observed));
  const bool marker = !sample.sequential && !sample.taken;
  expected.cpu.pc = target + 6;
  expected.cpu.x = (expected.cpu.x & ~word_mask) | (marker ? literal : 0U);
  expected.native_instructions += 16 + (marker ? 1U : 0U);
  expected.native_cycles += base_native_cycles + (marker ? 2U : 0U);
  require(
      machine.cpu().debug_state() == expected &&
          machine.bus().read({.space = DspSpace::x, .address = data}) == x_canary &&
          machine.bus().read({.space = DspSpace::y, .address = data}) == y_canary,
      "Quiet-window PCs, true path, full registers/flags/native counts and data canaries agree");
}

void independent_windows() {
  for (const auto sample : {WindowCase{.operation = "jset",
                                       .bit = 3,
                                       .taken = false,
                                       .sequential = false,
                                       .expected_clocks = 13},
                            WindowCase{.operation = "jclr",
                                       .bit = 3,
                                       .taken = true,
                                       .sequential = false,
                                       .expected_clocks = 10},
                            WindowCase{.operation = "jset",
                                       .bit = 2,
                                       .taken = true,
                                       .sequential = false,
                                       .expected_clocks = 10},
                            WindowCase{.operation = "jclr",
                                       .bit = 2,
                                       .taken = false,
                                       .sequential = false,
                                       .expected_clocks = 13},
                            WindowCase{.operation = "jset",
                                       .bit = 2,
                                       .taken = true,
                                       .sequential = true,
                                       .expected_clocks = 10},
                            WindowCase{.operation = "jclr",
                                       .bit = 2,
                                       .taken = false,
                                       .sequential = true,
                                       .expected_clocks = 11}}) {
    independent_window(sample);
  }
}
}

void branches() {
  independent_encodings();
  independent_windows();
  for (const auto sample : {BranchCase{.operation = "jset", .bit = 3, .taken = false},
                            BranchCase{.operation = "jclr", .bit = 3, .taken = true},
                            BranchCase{.operation = "jset", .bit = 2, .taken = true},
                            BranchCase{.operation = "jclr", .bit = 2, .taken = false}}) {
    actual_branch(sample);
  }
  discarded_path_has_no_effects();
  sequential_target(false);
  sequential_target(true);
  repeated_self_branch();
}
}
