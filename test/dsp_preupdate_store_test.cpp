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
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100, data = 0x407, canary = 0x654321;
constexpr unsigned ordinary_omr = 0x300, modifier = 0xff0007, absolute_jump = 0x0af080;
constexpr unsigned indexed_pointer = 0x404, predecrement_pointer = 0x400;
constexpr unsigned source_count = 12, accumulator_begin = 4, extension_begin = 6;
constexpr unsigned middle_begin = 8, full_begin = 10, extension_mask = 0xff;
constexpr unsigned upper_field_mask = 0x18, upper_field_shift = 17, full_flags = 0xff;
constexpr std::uint64_t a_seed = 0x12a1b2c3d4e5f6, b_seed = 0xe93f4a5b6c7d8e, data_seed = 0x111111;
constexpr std::array names{"x0", "x1", "y0", "y1", "a0", "b0", "a2", "b2", "a1", "b1", "a", "b"};
constexpr std::array<std::uint64_t, 8> values{0,
                                              1,
                                              0x007fffff123456,
                                              0x00800000654321,
                                              0x7f7fffff123456,
                                              0x80800000654321,
                                              0xff800000000001,
                                              0xffffffffffabcd};
constexpr std::array flags{0U, 0x3fU, 0xc0U, full_flags};

struct Case {
  unsigned source = 0, mode = 5, agu = 0, scale = 0, ccr = full_flags;
  std::uint64_t value = 0x80800000654321;
  bool y = false;
};

unsigned pointer(const Case input) {
  return input.mode == 5 ? indexed_pointer : predecrement_pointer;
}

unsigned updated(const Case input) {
  return input.mode == 5 ? indexed_pointer : data;
}

unsigned opcode(const Case input) {
  // FM13-118/122/Table12-14: standalone 01ddSddd,W=0,EA=1.
  constexpr unsigned base = 0x404000;
  constexpr unsigned y_bit = 0x80000;
  const auto field = input.source + 4;
  return base | ((field & upper_field_mask) << upper_field_shift) | ((field & 7U) << 16U) |
         (input.mode << 11U) | (input.agu << 8U) | (input.y ? y_bit : 0U);
}

void encoding(const Case input) {
  const auto reg = std::to_string(input.agu);
  const auto address = input.mode == 5 ? "(r" + reg + "+n" + reg + ')' : "-(r" + reg + ')';
  const auto text =
      std::string("move ") + names.at(input.source) + ',' + (input.y ? "y:" : "x:") + address;
  const auto actual = checkpoint::assemble(text.c_str());
  require(actual.size() == 1 && actual.front() == opcode(input),
          "Own store input matches the independently encoded primary source/address fields: " +
              text);
}

std::uint64_t initial_accumulator(const Case input, const bool b) {
  if (input.source >= accumulator_begin && (input.source % 2 != 0) == b) { return input.value; }
  return b ? b_seed : a_seed;
}

scalar_store::Result stored(const Case input) {
  if (input.source < accumulator_begin) {
    return {.value = static_cast<unsigned>(input.value & word_mask), .flags = input.ccr};
  }
  if (input.source >= full_begin) {
    // Independent signed56/scaling/24-bit-limiter/growth-bit oracle from FM3.5.
    return scalar_store::expected({.value = input.value, .scale = input.scale, .flags = input.ccr});
  }
  unsigned shift = 0;
  if (input.source >= middle_begin) {
    shift = word_bits;
  } else if (input.source >= extension_begin) {
    shift = data_bits;
  }
  const auto mask = shift == data_bits ? extension_mask : word_mask;
  auto value = static_cast<unsigned>((input.value >> shift) & mask);
  constexpr unsigned extension_sign = 0x80;
  constexpr unsigned extension_fill = 0xffff00;
  if (shift == data_bits && (value & extension_sign) != 0) { value |= extension_fill; }
  return {.value = value, .flags = input.ccr};
}

DspDebugState prepare(DspMachine &machine, const Case input) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  const auto reg = std::to_string(input.agu);
  program.immediate("m" + reg, modifier);
  program.immediate("r" + reg, pointer(input));
  program.immediate("n" + reg, 3);
  for (unsigned index = 0; index < 4; ++index) {
    program.immediate(names[index], data_seed * (index + 1));
  }
  if (input.source < accumulator_begin) {
    program.immediate(names.at(input.source), input.value & word_mask);
  }
  for (const bool b : {false, true}) {
    const auto value = initial_accumulator(input, b);
    const std::string name = b ? "b" : "a";
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(name + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status | input.scale | input.ccr);
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
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.r.at(input.agu) == pointer(input) &&
              before.cpu.n.at(input.agu) == 3 && before.cpu.m.at(input.agu) == modifier &&
              before.cpu.omr == ordinary_omr && before.cpu.a == initial_accumulator(input, false) &&
              before.cpu.b == initial_accumulator(input, true) &&
              before.cpu.sr == (masked_status | input.scale | input.ccr),
          "Actual native setup establishes full source/other accumulators and address/CCR inputs");
  return before;
}

void cells(DspMachine &machine, const Case input, const bool committed) {
  const auto value = stored(input).value;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected = committed && address == data && (space == DspSpace::y) == input.y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? value : canary),
              "Stores publish only the actual selected word and retain neighboring/opposite-space "
              "cells");
    }
  }
}

DspDebugState expected(DspDebugState before, const Case input, const std::uint64_t cycle) {
  if (cycle >= 5) { before.cpu.r.at(input.agu) = updated(input); }
  if (cycle == 8) {
    before.cpu.sr = masked_status | input.scale | stored(input).flags;
    ++before.cpu.pc;
    ++before.native_instructions;
    before.native_cycles += 2;
  }
  return before;
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
    std::cerr << "source=" << names.at(input.source) << " mode=" << input.mode
              << " agu=" << input.agu << " clock=" << cycle.completed_cycles
              << " input=" << hex(input.value, 14) << " sr=" << hex(actual.cpu.sr, 6) << '/'
              << hex(wanted.cpu.sr, 6) << '\n';
  }
  require(actual == wanted,
          "Independent full CPU/CCR/source preservation through AG2,held-E1,store and E2");
  if (!cycle.pipeline) { throw Fault("An automatic store lost its actual stage record"); }
  const auto &phase = *cycle.pipeline;
  require(phase.advanced == (cycle.completed_cycles != 6) && !phase.memory_source_stall &&
              phase.memory.has_value() == (cycle.completed_cycles == 7),
          "Every source uses one E1 hold and exactly one real native store");
  if (phase.memory) {
    const auto &memory = *phase.memory;
    require(memory.data.kind == DspNativeAccessKind::write &&
                memory.data.value == stored(input).value && memory.data.target.address == data &&
                memory.data.target.space == (input.y ? DspSpace::y : DspSpace::x) &&
                memory.data.pc == entry && memory.data.omr == ordinary_omr &&
                !memory.additional_data,
            "The actual native store retains exact source limiting/sign/space/address/PC/OMR");
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
          "Independent native full/partial/data store CPU/CCR/address/two-clock result");
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

unsigned addresses() {
  unsigned count = 0;
  for (unsigned source = 0; source < source_count; ++source) {
    for (const unsigned mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        for (unsigned agu = 0; agu < 8; ++agu) {
          const Case input{.source = source, .mode = mode, .agu = agu, .y = y};
          encoding(input);
          automatic(input);
          ++count;
        }
      }
    }
  }
  return count;
}

unsigned value_cases() {
  unsigned count = 0;
  for (unsigned source = 0; source < source_count; ++source) {
    for (const auto value : values) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const unsigned ccr : flags) {
          for (const unsigned mode : {5U, 7U}) {
            for (const bool y : {false, true}) {
              automatic({.source = source,
                         .mode = mode,
                         .agu = 7,
                         .scale = scale,
                         .ccr = ccr,
                         .value = value,
                         .y = y});
              ++count;
            }
          }
        }
      }
    }
  }
  return count;
}

void source_matrix(const bool atomic) {
  for (unsigned source = 0; source < source_count; ++source) {
    for (const unsigned mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        const Case input{.source = source, .mode = mode, .agu = 7, .scale = scale_up, .y = y};
        if (atomic) {
          native(input);
        } else {
          snapshots(input);
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
      throw Fault("Usage: store-probe [--staged|--native]");
    }
    if (mode == "--native") {
      source_matrix(true);
      std::cout << "48 native all-source store cases passed\n";
    } else {
      const auto addressing = addresses();
      const auto values_count = value_cases();
      source_matrix(false);
      std::cout << addressing << " all-source store addresses," << values_count
                << " value/scaling/CCR cases,384 cuts/768 suffixes passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
