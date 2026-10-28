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

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, data = 0x407, canary = 0x654321, loaded_word = 0x200000;
constexpr unsigned ordinary_omr = 0x300, modifier = 0xff0007, absolute_jump = 0x0af080;
constexpr unsigned indexed_pointer = 0x404, predecrement_pointer = 0x400;
constexpr unsigned data_targets = 4, r_targets = 12;
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

constexpr unsigned following_mac = 0x2000d2;
constexpr std::uint64_t new_x_product_sum = 0x191a2b000000;
constexpr std::uint64_t new_y_product_sum = 0x0da740800000;

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare_window(DspMachine &machine, const Case input, const unsigned gap) {
  encoding(input);
  const auto before = prepare(machine, input);
  machine.bus().write({.address = entry + 1 + gap, .value = following_mac});
  return before;
}

DspDebugState expected_window(const DspDebugState &before,
                              const Case input,
                              const unsigned gap,
                              const std::uint64_t count) {
  const auto first_cycle = count >= 8 ? std::uint64_t{8} : count;
  auto wanted = expected(before, input, first_cycle);
  if (gap != 0 && count >= 9) {
    ++wanted.cpu.pc;
    ++wanted.native_instructions;
    ++wanted.native_cycles;
  }
  if (count == 9 + gap) {
    wanted.cpu.a = input.target == 0 ? new_x_product_sum : new_y_product_sum;
    wanted.cpu.sr = masked_status | unnormalized | input.ccr;
    ++wanted.cpu.pc;
    ++wanted.native_instructions;
    ++wanted.native_cycles;
  }
  return wanted;
}

void period(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const Case input,
            const unsigned gap) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto count = cycle.completed_cycles;
  const auto wanted = expected_window(before, input, gap, count);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "target=" << names.at(input.target) << " mode=" << input.mode << " gap=" << gap
              << " clock=" << count << " A=" << hex(actual.cpu.a, 14) << '/'
              << hex(wanted.cpu.a, 14) << " SR=" << hex(actual.cpu.sr, 6) << '/'
              << hex(wanted.cpu.sr, 6) << " PC=" << hex(actual.cpu.pc, 6) << '/'
              << hex(wanted.cpu.pc, 6) << '\n';
  }
  require(actual == wanted,
          "Independent complete CPU/CCR verifies old parallel MPY inputs,then new MAC inputs and "
          "older A result");
  if (!cycle.pipeline) { throw Fault("A parallel ALU window lost its real stage record"); }
  const auto &phase = *cycle.pipeline;
  require(
      phase.advanced == (count != 6) && !phase.memory_source_stall &&
          phase.memory.has_value() == (count == 7),
      "The parallel word adds exactly one E1 hold;its pure-ALU successor keeps nominal throughput");
  if (phase.execute1 && phase.execute1->address == entry + 1 + gap) {
    require(phase.e1_deferred_accumulators == (gap == 0 ? 1U : 0U),
            "Only the adjacent actual older MPY supplies the MAC A dependency at E2");
  }
  cells(machine, input, count >= 7);
}

void one(const Case input, const unsigned gap) {
  DspMachine machine;
  const auto before = prepare_window(machine, input, gap);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 9 + gap; ++index) { period(run, machine, before, input, gap); }
}

void snapshots(const Case input, const unsigned gap) {
  for (unsigned cut = 0; cut < 9 + gap; ++cut) {
    DspMachine machine;
    const auto before = prepare_window(machine, input, gap);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned index = 0; index < cut; ++index) { period(run, machine, before, input, gap); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned index = cut; index < 9 + gap; ++index) {
        period(run, machine, before, input, gap);
      }
    }
  }
}

unsigned matrix() {
  unsigned count = 0;
  for (const unsigned mode : {5U, 7U}) {
    for (const bool y : {false, true}) {
      for (const unsigned target : {0U, 2U}) {
        for (const unsigned gap : {0U, 1U}) {
          for (unsigned agu = 0; agu < 8; ++agu) {
            for (const unsigned ccr : {0U, carry}) {
              one({.mode = mode, .agu = agu, .target = target, .ccr = ccr, .y = y}, gap);
              ++count;
            }
          }
          snapshots({.mode = mode, .agu = 7, .target = target, .ccr = carry, .y = y}, gap);
        }
      }
    }
  }
  return count;
}
}

int main() {
  try {
    const auto count = matrix();
    std::cout << count << " parallel-MPY/new-data-MAC windows,152 cuts/304 suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
