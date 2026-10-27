// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned seed_x1 = 0x45f400;
constexpr unsigned seed_r0 = 0x60f400;
constexpr unsigned direct = 0x0ae080;
constexpr unsigned absolute = 0x0af080;
constexpr unsigned switched_omr = 0x389;
constexpr unsigned body = boot + 6;
constexpr unsigned writer = body + 1;
constexpr unsigned branch = body + 2;
constexpr unsigned target = body + 6;
constexpr unsigned marker = 0x654321;
constexpr unsigned marker_opcode = 0x46f400;
constexpr unsigned omr_opcode = 0x04c5ba;
constexpr unsigned tail = 32;
constexpr unsigned branch_cost = 4;
constexpr unsigned branch_e2 = 9;
constexpr unsigned taken_successor = 13;
constexpr unsigned false_marker = 14;
constexpr unsigned false_successor = 16;

struct Case {
  unsigned opcode;
  unsigned words;
  bool taken;
  unsigned omr;
};

DspDebugState prepare(DspMachine &machine, const Case input) {
  DspProgram code;
  code.word(sr_move);
  code.word(masked_status);
  code.word(seed_x1);
  code.word(input.omr);
  code.word(seed_r0);
  code.word(target);
  code.word(0);
  code.word(omr_opcode);
  code.word(input.opcode);
  code.word(input.words == 2 ? target : 0U);
  code.word(marker_opcode);
  code.word(marker);
  for (unsigned index = 0; index < tail; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 3);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.omr == reset_omr,
          "Real native own seeds retain the old OMR before the clocked stream");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspFetchRunInputs contract() {
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  return input;
}

struct Oracle {
  DspDebugState expected;
  unsigned retired = 0;
};

void step(DspFetchRun &run, DspMachine &machine, const Case input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  if (!input.taken && cycle.completed_cycles == false_marker - 1) {
    require(phase.execute1 && phase.execute1->address == branch + 2,
            "The false path's genuine long-immediate MOVE publishes its Y field in E1");
    oracle.expected.cpu.y = (oracle.expected.cpu.y & ~std::uint64_t{word_mask}) | marker;
  }
  if (phase.retirement_published) {
    if (!phase.execute2) { throw Fault("Native E2 lost the actual instruction word"); }
    require(phase.execute2->address == oracle.expected.cpu.pc,
            "Every genuine E2 follows the expected new-OMR branch path");
    unsigned cost = 1;
    const auto pc = phase.execute2->address;
    if (pc == branch) {
      require(cycle.completed_cycles == branch_e2,
              "The older OMR instruction adds no synthetic branch issue delay");
      oracle.expected.cpu.pc = input.taken ? target : branch + input.words;
      cost = input.opcode == direct ? 3U : branch_cost;
    } else {
      if (pc == writer) { oracle.expected.cpu.omr = input.omr; }
      if (pc == branch + 2 && !input.taken) {
        require(cycle.completed_cycles == false_marker,
                "False branch retains its independently documented five-clock issue window");
        oracle.expected.cpu.y = (oracle.expected.cpu.y & ~std::uint64_t{word_mask}) | marker;
        cost = 2;
      }
      oracle.expected.cpu.pc += cost;
    }
    ++oracle.expected.native_instructions;
    oracle.expected.native_cycles += cost;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every branch period preserves whole literal CPU, frame and native counter state");
  require(machine.reference_fetch().omr == oracle.expected.cpu.omr,
          "Logical execution context follows the actual native OMR publication");
}

void one_case(const Case input, const std::optional<unsigned> cut = std::nullopt) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input), .retired = 0};
  DspFetchRun run(contract(), machine);
  auto end = input.taken ? taken_successor : false_successor;
  if (input.opcode == direct) { --end; }
  const auto check = [&] {
    require(oracle.retired == (input.taken ? 4U : 5U) && machine.cpu().state().pc == target + 1 &&
                run.elapsed_ticks() == end * pipeline::period,
            "Actual new-context branch and its successor compose without extra OMR stalls");
  };
  if (cut) {
    while (run.completed_cycles() < *cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix;
      while (run.completed_cycles() < end) { step(run, machine, input, oracle); }
      check();
    }
  } else {
    while (run.completed_cycles() < end) { step(run, machine, input, oracle); }
    check();
  }
}
}

int main() {
  try {
    constexpr unsigned relative = 0x050c04;
    constexpr unsigned jset = 0x0afa27;
    constexpr unsigned jclr = 0x0afa07;
    for (const auto value : {0x305U, 0x389U, 0x600389U}) {
      const bool set = (value & 0x80U) != 0;
      for (const auto input : {Case{.opcode = direct, .words = 1, .taken = true, .omr = value},
                               Case{.opcode = relative, .words = 1, .taken = true, .omr = value},
                               Case{.opcode = absolute, .words = 2, .taken = true, .omr = value},
                               Case{.opcode = jset, .words = 2, .taken = set, .omr = value},
                               Case{.opcode = jclr, .words = 2, .taken = !set, .omr = value}}) {
        one_case(input);
      }
    }
    for (const auto opcode : {jset, jclr, absolute}) {
      for (unsigned cut = 4; cut <= 9; ++cut) {
        one_case({.opcode = opcode, .words = 2, .taken = opcode != jclr, .omr = switched_omr}, cut);
      }
    }
    std::cout << "15 OMR-to-branch streams and 36 trusted suffixes passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
