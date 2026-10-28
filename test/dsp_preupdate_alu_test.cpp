// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, data = 0x407, canary = 0x654321, loaded_word = 0x200000;
constexpr unsigned ordinary_omr = 0x300, modifier = 0xff0007, absolute_jump = 0x0af080;
constexpr unsigned indexed_pointer = 0x404, predecrement_pointer = 0x400;
constexpr unsigned data_targets = 4, r_targets = 12, target_count = 20;
constexpr unsigned upper_field_mask = 0x18, upper_field_shift = 17, y_bit = 0x80000;
constexpr unsigned load_base = 0x40c000, store_base = 0x404000;
constexpr std::uint64_t a_seed = 0x200000000000, b_seed = 0x1000000, pair_mask = 0xffffffffffff;
constexpr std::array names{"x0", "x1", "y0", "y1", "r0", "r1", "r2", "r3", "r4", "r5",
                           "r6", "r7", "n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7"};
constexpr std::array original_data{0x123456U, 1U, 0x400000U, 2U};

struct Operation {
  const char *text;
  unsigned bits;
  std::uint64_t result;
  unsigned flags;
  bool retains_carry;
};

// FM instruction/CCR definitions: literal old-operand MPY/MAC and ADD/CLR.
// Product=2*$123456*$400000; ADD adds oldX1<<24,not the parallel load.
constexpr std::array operations{Operation{.text = "mpy y0,x0,a",
                                          .bits = 0xd0,
                                          .result = 0x091a2b000000,
                                          .flags = unnormalized,
                                          .retains_carry = true},
                                Operation{.text = "mac y0,x0,a",
                                          .bits = 0xd2,
                                          .result = 0x291a2b000000,
                                          .flags = unnormalized,
                                          .retains_carry = true},
                                Operation{.text = "add x1,a",
                                          .bits = 0x60,
                                          .result = 0x200001000000,
                                          .flags = unnormalized,
                                          .retains_carry = false},
                                Operation{.text = "clr a",
                                          .bits = 0x13,
                                          .result = 0,
                                          .flags = unnormalized | zero,
                                          .retains_carry = true}};

struct Case {
  unsigned operation = 0, mode = 5, agu = 0, target = 0, ccr = 0;
  bool y = false, load = true;
};

unsigned pointer(const Case input) {
  return input.mode == 5 ? indexed_pointer : predecrement_pointer;
}

unsigned updated(const Case input) {
  return input.mode == 5 ? indexed_pointer : data;
}

unsigned opcode(const Case input) {
  // FM13-118/122/Table12-14: data or R/N destinations,with the literal ALU byte.
  const auto field = input.target < data_targets ? input.target + 4 : input.target + r_targets;
  return (input.load ? load_base : store_base) | ((field & upper_field_mask) << upper_field_shift) |
         ((field & 7U) << 16U) | (input.mode << 11U) | (input.agu << 8U) | (input.y ? y_bit : 0U) |
         operations.at(input.operation).bits;
}

void encoding(const Case input) {
  const auto reg = std::to_string(input.agu);
  const auto address = input.mode == 5 ? "(r" + reg + "+n" + reg + ')' : "-(r" + reg + ')';
  const auto memory = std::string(input.y ? "y:" : "x:") + address;
  const auto operand = input.load ? memory + ',' + names.at(input.target)
                                  : std::string(names.at(input.target)) + ',' + memory;
  const auto text = std::string(operations.at(input.operation).text) + ' ' + operand;
  const auto actual = checkpoint::assemble(text.c_str());
  require(actual.size() == 1 && actual.front() == opcode(input),
          "Parallel fields must match independent primary ALU/memory encoding: " + text);
}

DspDebugState prepare(DspMachine &machine, const Case input) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  const auto reg = std::to_string(input.agu);
  program.immediate("m" + reg, modifier);
  program.immediate("r" + reg, pointer(input));
  program.immediate("n" + reg, 3);
  for (unsigned index = 0; index < original_data.size(); ++index) {
    program.immediate(names[index], original_data[index]);
  }
  for (const bool b : {false, true}) {
    const auto value = b ? b_seed : a_seed;
    const std::string name = b ? "b" : "a";
    for (unsigned part = 0; part < 3; ++part) {
      constexpr unsigned extension_mask = 0xff;
      program.immediate(name + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status | input.ccr);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = opcode(input)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected = input.load && address == data && (space == DspSpace::y) == input.y;
      machine.bus().write(
          {.space = space, .address = address, .value = selected ? loaded_word : canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  constexpr std::uint64_t x_seed = 0x1123456;
  constexpr std::uint64_t y_seed = 0x2400000;
  require(
      before.cpu.pc == entry && before.cpu.r.at(input.agu) == pointer(input) &&
          before.cpu.n.at(input.agu) == 3 && before.cpu.m.at(input.agu) == modifier &&
          before.cpu.omr == ordinary_omr && before.cpu.a == a_seed && before.cpu.b == b_seed &&
          before.cpu.x == x_seed && before.cpu.y == y_seed &&
          before.cpu.sr == (masked_status | input.ccr),
      "Real native setup establishes literal ALU and move inputs before the shared instruction");
  return before;
}

DspDebugState moved(DspDebugState state, const Case input) {
  if (input.target < data_targets) {
    auto &pair = input.target < 2 ? state.cpu.x : state.cpu.y;
    const auto shift = (input.target % 2) * word_bits;
    const auto mask = static_cast<std::uint64_t>(word_mask) << shift;
    pair = (pair & (pair_mask ^ mask)) | (static_cast<std::uint64_t>(loaded_word) << shift);
  } else if (input.target < r_targets) {
    state.cpu.r.at(input.target - data_targets) = loaded_word;
  } else {
    state.cpu.n.at(input.target - r_targets) = loaded_word;
  }
  return state;
}

DspDebugState expected(DspDebugState before, const Case input, const std::uint64_t cycle) {
  if (cycle >= 5) { before.cpu.r.at(input.agu) = updated(input); }
  if (cycle >= 7 && input.load) { before = moved(before, input); }
  if (cycle == 8) {
    const auto op = operations.at(input.operation);
    before.cpu.a = op.result;
    before.cpu.sr = masked_status | op.flags | (op.retains_carry ? input.ccr : 0U);
    ++before.cpu.pc;
    ++before.native_instructions;
    before.native_cycles += 2;
  }
  return before;
}

void cells(DspMachine &machine, const Case input, const bool committed) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      auto value = canary;
      if (address == data && (space == DspSpace::y) == input.y) {
        if (input.load) { value = loaded_word; }
        if (!input.load && committed) { value = original_data.at(input.target); }
      }
      require(
          machine.bus().read({.space = space, .address = address}) == value,
          "Parallel native memory effects retain exact source,neighbors and opposite-space cells");
    }
  }
}

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void period(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case input) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto wanted = expected(before, input, cycle.completed_cycles);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "op=" << input.operation << " target=" << names.at(input.target)
              << " load=" << input.load << " mode=" << input.mode
              << " clock=" << cycle.completed_cycles << " opcode=" << hex(opcode(input), 6)
              << " A=" << hex(actual.cpu.a, 14) << '/' << hex(wanted.cpu.a, 14)
              << " SR=" << hex(actual.cpu.sr, 6) << '/' << hex(wanted.cpu.sr, 6) << '\n';
  }
  require(actual == wanted,
          "Independent complete CPU/CCR verifies old ALU operands and only real E1/E2 publication");
  if (!cycle.pipeline) { throw Fault("A parallel pre-update word lost its actual stage record"); }
  const auto &phase = *cycle.pipeline;
  require(
      phase.advanced == (cycle.completed_cycles != 6) && !phase.memory_source_stall &&
          phase.memory.has_value() == (cycle.completed_cycles == 7),
      "Parallel arithmetic shares the exact owned extra E1 period and single native memory effect");
  if (phase.memory) {
    const auto &memory = *phase.memory;
    const auto kind = input.load ? DspNativeAccessKind::read : DspNativeAccessKind::write;
    const auto value = input.load ? loaded_word : original_data.at(input.target);
    require(memory.data.kind == kind && memory.data.value == value &&
                memory.data.target.address == data &&
                memory.data.target.space == (input.y ? DspSpace::y : DspSpace::x) &&
                !memory.additional_data,
            "Parallel move reads/writes its actual literal word without altering the ALU's "
            "original operands");
  }
  cells(machine, input, cycle.completed_cycles >= 7);
}

void automatic(const Case input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 8; ++index) { period(run, machine, before, input); }
}

void native(const Case input) {
  DspMachine machine;
  const auto wanted = expected(prepare(machine, input), input, 8);
  machine.cpu().step();
  require(machine.cpu().debug_state() == wanted,
          "Independent native parallel ALU/move result,CCR and two-clock cost");
  cells(machine, input, true);
}

void snapshots(const Case input) {
  for (unsigned cut = 0; cut < 8; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, input);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned index = 0; index < cut; ++index) { period(run, machine, before, input); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned index = cut; index < 8; ++index) { period(run, machine, before, input); }
    }
  }
}

unsigned targets(Case input, const bool atomic) {
  unsigned count = 0;
  for (unsigned target = 0; target < target_count; ++target) {
    input.target = target;
    input.load = true;
    encoding(input);
    if (atomic) {
      native(input);
    } else {
      automatic(input);
    }
    ++count;
  }
  for (unsigned target = 0; target < data_targets; ++target) {
    input.target = target;
    input.load = false;
    encoding(input);
    if (atomic) {
      native(input);
    } else {
      automatic(input);
    }
    ++count;
  }
  return count;
}

unsigned matrix(const bool atomic) {
  unsigned count = 0;
  const unsigned agu_count = atomic ? 1U : 8U;
  for (unsigned op = 0; op < operations.size(); ++op) {
    for (const unsigned mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        for (unsigned agu = 0; agu < agu_count; ++agu) {
          for (const unsigned ccr : {0U, carry}) {
            count +=
                targets({.operation = op, .mode = mode, .agu = agu, .ccr = ccr, .y = y}, atomic);
          }
        }
      }
    }
  }
  return count;
}

void snapshot_matrix() {
  for (unsigned op = 0; op < operations.size(); ++op) {
    for (const unsigned mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        for (const unsigned target : {0U, 2U, 4U, 12U}) {
          snapshots(
              {.operation = op, .mode = mode, .agu = 0, .target = target, .ccr = carry, .y = y});
        }
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--staged";
    if (count > 2 || (mode != "--staged" && mode != "--native")) {
      throw Fault("Usage: alu-probe [--staged|--native]");
    }
    const bool atomic = mode == "--native";
    const auto cases = matrix(atomic);
    if (atomic) {
      std::cout << cases << " native parallel ALU/move cases passed\n";
    } else {
      snapshot_matrix();
      std::cout << cases << " staged parallel ALU/move cases,512 cuts/1024 suffixes passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
