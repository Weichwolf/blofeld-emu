// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_accumulator_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::parallel_accumulator;

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void complete(DspDebugState &expected, const Input input) {
  const auto sum = addition(input.value, input.status);
  expected.cpu.a = input.b ? sum.value : loaded(input.word);
  expected.cpu.b = input.b ? loaded(input.word) : sum.value;
  expected.cpu.sr = sum.status;
  expected.cpu.r.at(input.reg) = updated_pointer(input);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  expected.native_cycles += 1 + (input.mode == 5 || input.mode == 7 ? 1U : 0U);
}

void encoding(const Input input) {
  const auto reg = "r" + std::to_string(input.reg);
  const auto offset_reg = "n" + std::to_string(input.reg);
  const std::array ea{'(' + reg + ")-" + offset_reg,
                      '(' + reg + ")+" + offset_reg,
                      '(' + reg + ")-",
                      '(' + reg + ")+",
                      '(' + reg + ')',
                      '(' + reg + '+' + offset_reg + ')',
                      std::string{},
                      "-(" + reg + ')'};
  const auto text = std::string(input.b ? "add b,a " : "add a,b ") + (input.y ? "y:" : "x:") +
                    ea.at(input.mode) + (input.b ? ",b" : ",a");
  const auto words = checkpoint::assemble(text.c_str());
  require(words.size() == 1 && words.front() == opcode(input),
          "Own assembler must agree with independent primary ADD/load fields");
}

void native(const Input input) {
  DspMachine machine;
  const std::array code{opcode(input)};
  auto expected = prepare(machine, input, code);
  complete(expected, input);
  machine.cpu().step();
  require(machine.cpu().debug_state() == expected,
          "Native full state follows independent old-operand ADD/load/flags/address expectations");
  sources(machine, input);
}

void staged(const Input input) {
  DspMachine machine;
  const std::array code{opcode(input)};
  const auto before = prepare(machine, input, code);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  const unsigned read_clock = 6 + (input.mode == 5 || input.mode == 7 ? 1U : 0U);
  unsigned reads = 0;
  for (unsigned cycle = 1; cycle <= read_clock + 1; ++cycle) {
    const auto result = run.advance_cycle(machine, fetch_clock::input());
    auto expected = before;
    if (cycle >= 5) { expected.cpu.r.at(input.reg) = updated_pointer(input); }
    if (cycle >= read_clock) { (input.b ? expected.cpu.b : expected.cpu.a) = loaded(input.word); }
    if (cycle > read_clock) { complete(expected, input); }
    require(machine.cpu().debug_state() == expected,
            "AG2 pointer/E1 full load/E2 old-operand ADD and flags publish at distinct periods");
    if (!result.bus.memory) { continue; }
    ++reads;
    const auto &memory = *result.bus.memory;
    const auto fields = input.b ? DspMoveFields{.b = 7} : DspMoveFields{.a = 7};
    require(cycle == read_clock && memory.data.kind == DspNativeAccessKind::read &&
                memory.data.target.space == (input.y ? DspSpace::y : DspSpace::x) &&
                memory.data.target.address == address(input) && memory.data.value == input.word &&
                memory.move && memory.move->fields == fields && !memory.additional_data,
            "E1 publishes exactly one actual source and its distinct full-accumulator load");
  }
  require(reads == 1 && !run.faulted(), "Only one actual data read may occur");
  sources(machine, input);
}

void variants(const auto &action, const Pair value, const unsigned word, const unsigned ccr) {
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const unsigned sm : {0U, saturation_mode}) {
          action(Input{.value = value,
                       .word = word,
                       .status = masked_status | scale | sm | ccr,
                       .b = b,
                       .y = y});
        }
      }
    }
  }
}

void matrix(const auto &action) {
  constexpr std::array words{0U, 1U, 0x800000U, word_mask};
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) { encoding({.value = {}, .b = b, .y = y}); }
  }
  for (unsigned pair = 0; pair < operands.size(); ++pair) {
    for (unsigned word = 0; word < words.size(); ++word) {
      variants(action, operands.at(pair), words.at(word), (pair + word) % 2 != 0 ? all_ccr : 0U);
    }
  }
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (unsigned reg = 0; reg < 8; ++reg) {
        for (const unsigned mode : {0U, 1U, 2U, 3U, 4U, 5U, 7U}) {
          if (reg == 4 && mode == 3) { continue; }
          const Input input{.value = operands.at(6),
                            .status = masked_status | all_ccr,
                            .reg = reg,
                            .mode = mode,
                            .b = b,
                            .y = y};
          encoding(input);
          action(input);
        }
      }
    }
  }
}
}

int main(const int argc, const char *const *argv) {
  try {
    require(argc == 2, "Select native,pipeline,dependencies,state or guards");
    const std::string_view mode = argv[1];
    if (mode == "native") {
      matrix(native);
    } else if (mode == "pipeline") {
      matrix(staged);
    } else if (mode == "dependencies") {
      dependencies();
    } else if (mode == "state") {
      checkpoints();
    } else {
      require(mode == "guards", "Unknown ADD/load scope");
      guards();
    }
    std::cout << "Independent full-accumulator ADD/load " << mode << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
