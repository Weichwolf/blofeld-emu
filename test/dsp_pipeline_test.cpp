// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pipeline.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::pipeline;

std::optional<unsigned> stage_pc(const std::optional<DspProgramWord> &word) {
  return word ? std::optional{word->address} : std::nullopt;
}

std::optional<unsigned> expected_pc(const unsigned cycle, const unsigned first) {
  return cycle >= first ? std::optional{body + cycle - first} : std::nullopt;
}

struct ClockIndex {
  unsigned cycle = 0;
  unsigned retired = 0;
  unsigned instructions = 0;
};

void verify_operands(const DspPipelineCycle &phase, const ClockIndex index) {
  if (phase.candidate_inputs && phase.e1_operands) {
    const auto prior = index.retired == 0 ? 0 : index.retired - 1;
    const auto deferred = index.retired != 0 && index.retired < index.instructions ? 1U : 0U;
    require(phase.e1_operands->a == prior * unit &&
                phase.candidate_inputs->a == index.retired * unit &&
                phase.e1_deferred_accumulators == deferred,
            "E1 register sampling stays distinct from the preceding-ALU operand deferred to E2" +
                std::string(" cycle=") + std::to_string(index.cycle) + " e1=" +
                number(phase.e1_operands->a) + " candidate=" + number(phase.candidate_inputs->a) +
                " deferred=" + std::to_string(phase.e1_deferred_accumulators));
  }
  if (phase.e2_deferred_accumulators != 0) {
    require(phase.e2_operands && phase.e2_operands->a == (index.retired - 1) * unit,
            "The deferred real accumulator is verified at E2 before result publication");
  }
}

void cold_fill_and_throughput() {
  DspMachine machine;
  constexpr unsigned instructions = 12;
  std::array<const char *, instructions> code{};
  code.fill("add x1,a");
  prepare(machine, code, {.timer = true});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned cycle = 1; cycle <= instructions + 6; ++cycle) {
    const auto result = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = stages(result);
    require(result.issued_address == body + cycle - 1 &&
                stage_pc(phase.decode) == expected_pc(cycle, 3) &&
                stage_pc(phase.ag1) == expected_pc(cycle, 4) &&
                stage_pc(phase.ag2) == expected_pc(cycle, 5) &&
                stage_pc(phase.execute1) == expected_pc(cycle, 6) &&
                stage_pc(phase.execute2) == expected_pc(cycle, 7) && phase.advanced,
            "Actual F1/F2/decode/AG1/AG2/E1/E2 advance without caller-selected phase tickets");
    const auto retired = cycle >= 7 ? cycle - 6 : 0;
    auto expected = before;
    expected.cpu.a = retired * unit;
    if (retired != 0) { expected.cpu.sr = masked_status | unnormalized; }
    expected.cpu.pc = body + retired;
    expected.native_instructions += retired;
    expected.native_cycles += retired;
    require(
        machine.cpu().debug_state() == expected && run.completed_cycles() == cycle &&
            run.elapsed_ticks() == cycle * period && run.timer_clk2_edges() == cycle / 2,
        "Literal ADD/CCR/counters publish first at clock7 and then once per real elapsed clock");
    require(result.bus.core_access_count == (cycle == 1 ? 0 : 1),
            "Only the actual F2 performs a P-bus read; native E1 consumes the captured word");
    verify_operands(phase, {.cycle = cycle, .retired = retired, .instructions = instructions});
    if (phase.execute2) {
      require(phase.retirement_published && result.bus.progress && result.bus.progress->retired,
              "E2 records a genuine native publication, not a staged trace counter");
    }
  }
  require((machine.bus().timers().read(fetch_clock::csr0, fetch_clock::timer_flag) &
           fetch_clock::timer_flag) != 0,
          "Real CLK2 edges during fill and execution set the configured timer flag");
}

void mixed_native_alu() {
  struct Expected {
    const char *instruction;
    std::uint64_t a;
    std::uint64_t b;
  };

  constexpr std::array expected{
      Expected{.instruction = "mpy y0,x0,a", .a = 0x00200000000000, .b = unit},
      Expected{.instruction = "add x1,a", .a = 0x00200001000000, .b = unit},
      Expected{.instruction = "mac y0,x0,a", .a = 0x00400001000000, .b = unit},
      Expected{.instruction = "neg a", .a = 0xffbfffff000000, .b = unit},
      Expected{.instruction = "abs a", .a = 0x00400001000000, .b = unit},
      Expected{.instruction = "asr a", .a = 0x00200000800000, .b = unit},
      Expected{.instruction = "asl a", .a = 0x00400001000000, .b = unit},
      Expected{.instruction = "tfr a,b", .a = 0x00400001000000, .b = 0x00400001000000},
      Expected{.instruction = "clr a", .a = 0, .b = 0x00400001000000},
      Expected{.instruction = "add b,a", .a = 0x00400001000000, .b = 0x00400001000000},
      Expected{.instruction = "sub x1,a", .a = 0x00400000000000, .b = 0x00400001000000},
      Expected{.instruction = "mpy y0,x0,b", .a = 0x00400000000000, .b = 0x00200000000000},
      Expected{.instruction = "mac y0,x0,b", .a = 0x00400000000000, .b = 0x00400000000000}};
  std::array<const char *, expected.size()> code{};
  for (unsigned index = 0; index < code.size(); ++index) {
    code[index] = expected[index].instruction;
  }
  DspMachine machine;
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  for (unsigned index = 0; index < code.size(); ++index) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto state = machine.cpu().state();
    require(
        stages(cycle).retirement_published && state.a == expected[index].a &&
            state.b == expected[index].b && state.pc == body + index + 1,
        "Independent literal signed fractional arithmetic follows the actual native ALU pipeline");
    if (index < 2) {
      const auto status = masked_status | unnormalized | (index == 0 ? carry : 0);
      require(state.sr == status, "Independent MPY/ADD flags accompany their actual E2 results");
    }
  }
}
}

int main() {
  try {
    cold_fill_and_throughput();
    mixed_native_alu();
    state_checks();
    guard_checks();
    std::cout << "Automatic seven-stage native ALU scheduling checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
