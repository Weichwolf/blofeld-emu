// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
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
constexpr unsigned entry = 0x100, data = 0x407, canary = 0x654321;
constexpr unsigned ordinary_omr = 0x300, modifier = 0xff0007, linear = 0xffffff;
constexpr unsigned absolute_jump = 0x0af080, full_flags = 0xff, extension_mask = 0xff;
constexpr unsigned scalar_fields = 28, target_count = 36, r_begin = 12, n_begin = 20, m_begin = 28;
constexpr unsigned indexed_pointer = 0x404, predecrement_pointer = 0x400, address_seed = 0x600;
constexpr unsigned upper_field_mask = 0x18, upper_field_shift = 17;
constexpr std::uint64_t data_seed = 0x111111, data_pair_mask = 0xffffffffffff;
constexpr std::uint64_t a_seed = 0x12a1b2c3d4e5f6, b_seed = 0xe93f4a5b6c7d8e;
constexpr std::array names{"x0", "x1", "y0", "y1", "a0", "b0", "a2", "b2", "a1", "b1", "a", "b"};
constexpr std::array values{0U, 1U, 0x7fU, 0x80U, 0x7fffffU, 0x800000U, 0xffff80U, 0xffffffU};
constexpr std::array flags{0U, 0x3fU, 0xc0U, full_flags};

struct Case {
  unsigned target = 0, mode = 5, agu = 0, value = 0x800001, ccr = full_flags, scale = 0;
  bool y = false;
};

unsigned pointer(const Case sample) {
  return sample.mode == 5 ? indexed_pointer : predecrement_pointer;
}

unsigned updated(const Case sample) {
  return sample.mode == 5 ? indexed_pointer : data;
}

std::string target_name(const unsigned target) {
  if (target < r_begin) { return names.at(target); }
  if (target < n_begin) { return "r" + std::to_string(target - r_begin); }
  if (target < m_begin) { return "n" + std::to_string(target - n_begin); }
  return "m" + std::to_string(target - m_begin);
}

unsigned opcode(const Case sample) {
  const auto ea = (sample.mode << 11U) | (sample.agu << 8U);
  // FM13-118/122,Table12-14: ddddd split as 01ddSddd,W=1,EA=1.
  if (sample.target < scalar_fields) {
    constexpr unsigned base = 0x40c000;
    constexpr unsigned y_bit = 0x80000;
    const auto field = sample.target + 4;
    return base | ((field & upper_field_mask) << upper_field_shift) | ((field & 7U) << 16U) | ea |
           (sample.y ? y_bit : 0U);
  }
  // FM13-134/Table12-20: MOVEC EA,Mn has DDDDD=000nnn; S is bit6.
  constexpr unsigned modifier_load = 0x05c020;
  constexpr unsigned control_y_bit = 0x40;
  return modifier_load | ea | (sample.target - m_begin) | (sample.y ? control_y_bit : 0U);
}

void encoding(const Case sample) {
  const auto reg = std::to_string(sample.agu);
  const auto address = sample.mode == 5 ? "(r" + reg + "+n" + reg + ')' : "-(r" + reg + ')';
  const auto text =
      std::string("move ") + (sample.y ? "y:" : "x:") + address + ',' + target_name(sample.target);
  const auto actual = checkpoint::assemble(text.c_str());
  require(actual.size() == 1 && actual.front() == opcode(sample),
          "Own native input must match independent primary-manual register/address encoding: " +
              text);
}

void cells(DspMachine &machine, const Case sample) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected = address == data && (space == DspSpace::y) == sample.y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? sample.value : canary),
              "Native loads retain both memory spaces and all neighboring canaries");
    }
  }
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  for (unsigned index = 0; index < 8; ++index) {
    const auto reg = std::to_string(index);
    program.immediate("r" + reg, address_seed + index);
    program.immediate("n" + reg, 0x10U + index);
    program.immediate("m" + reg, linear);
  }
  const auto reg = std::to_string(sample.agu);
  program.immediate("m" + reg, modifier);
  program.immediate("r" + reg, pointer(sample));
  program.immediate("n" + reg, 3);
  for (unsigned index = 0; index < 4; ++index) {
    program.immediate(names[index], data_seed * (index + 1));
  }
  for (const bool b : {false, true}) {
    const auto value = b ? b_seed : a_seed;
    const std::string name = b ? "b" : "a";
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(name + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status | sample.scale | sample.ccr);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned index = 0; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {data - 1, data, data + 1}) {
      const bool selected = address == data && (space == DspSpace::y) == sample.y;
      machine.bus().write(
          {.space = space, .address = address, .value = selected ? sample.value : canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.a == a_seed && before.cpu.b == b_seed &&
              before.cpu.r.at(sample.agu) == pointer(sample) && before.cpu.n.at(sample.agu) == 3 &&
              before.cpu.m.at(sample.agu) == modifier && before.cpu.omr == ordinary_omr &&
              before.cpu.sr == (masked_status | sample.scale | sample.ccr),
          "Real own native setup establishes both accumulators and actual address/CCR inputs");
  return before;
}

std::uint64_t accumulator(const std::uint64_t initial, const Case sample) {
  constexpr unsigned full_begin = 10;
  constexpr unsigned extension_begin = 6;
  constexpr unsigned middle_begin = 8;
  if (sample.target >= full_begin) {
    constexpr unsigned sign = 0x800000;
    constexpr std::uint64_t negative_extension = 0xff000000000000;
    return (static_cast<std::uint64_t>(sample.value) << word_bits) |
           ((sample.value & sign) != 0 ? negative_extension : 0);
  }
  unsigned shift = 0;
  if (sample.target >= middle_begin) {
    shift = word_bits;
  } else if (sample.target >= extension_begin) {
    shift = data_bits;
  }
  const auto width = shift == data_bits ? extension_mask : word_mask;
  const auto mask = static_cast<std::uint64_t>(width) << shift;
  return (initial & (accumulator_mask ^ mask)) |
         ((static_cast<std::uint64_t>(sample.value) << shift) & mask);
}

DspDebugState loaded(DspDebugState state, const Case sample) {
  constexpr unsigned accumulator_begin = 4;
  if (sample.target < accumulator_begin) {
    auto &pair = sample.target < 2 ? state.cpu.x : state.cpu.y;
    const auto shift = (sample.target % 2) * word_bits;
    const auto mask = static_cast<std::uint64_t>(word_mask) << shift;
    pair = (pair & (data_pair_mask ^ mask)) | (static_cast<std::uint64_t>(sample.value) << shift);
  } else if (sample.target < r_begin) {
    auto &target = sample.target % 2 != 0 ? state.cpu.b : state.cpu.a;
    target = accumulator(target, sample);
  } else if (sample.target < n_begin) {
    state.cpu.r.at(sample.target - r_begin) = sample.value;
  } else if (sample.target < m_begin) {
    state.cpu.n.at(sample.target - n_begin) = sample.value;
  } else {
    state.cpu.m.at(sample.target - m_begin) = sample.value;
  }
  return state;
}

DspDebugState expected(DspDebugState before, const Case sample, const std::uint64_t cycle) {
  if (cycle >= 5) { before.cpu.r.at(sample.agu) = updated(sample); }
  if (cycle >= 7) { before = loaded(before, sample); }
  if (cycle == 8) {
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

void period(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto wanted = expected(before, sample, cycle.completed_cycles);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "target=" << target_name(sample.target) << " mode=" << sample.mode
              << " agu=" << sample.agu << " clock=" << cycle.completed_cycles
              << " opcode=" << hex(opcode(sample), 6)
              << " R=" << hex(actual.cpu.r.at(sample.agu), 6) << '/'
              << hex(wanted.cpu.r.at(sample.agu), 6) << '\n';
  }
  require(
      actual == wanted,
      "Independent full CPU/CCR,part preservation and alias priority at every pre-update period");
  if (!cycle.pipeline) { throw Fault("An automatic register load lost its actual stage record"); }
  const auto &phase = *cycle.pipeline;
  require(phase.advanced == (cycle.completed_cycles != 6) && !phase.memory_source_stall &&
              phase.memory.has_value() == (cycle.completed_cycles == 7),
          "A destination-independent extra E1 clock retains exactly one native data operation");
  if (phase.memory) {
    const auto &read = phase.memory->data;
    require(read.kind == DspNativeAccessKind::read && read.value == sample.value &&
                read.target.address == data &&
                read.target.space == (sample.y ? DspSpace::y : DspSpace::x),
            "Every admitted destination consumes its real selected native memory word");
  }
  cells(machine, sample);
}

void automatic(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned index = 0; index < 8; ++index) { period(run, machine, before, sample); }
}

void native(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  const auto wanted = expected(before, sample, 8);
  machine.cpu().step();
  require(machine.cpu().debug_state() == wanted,
          "Independent atomic native destination/address/CCR/two-clock expectation");
  cells(machine, sample);
}

void snapshots(const Case sample) {
  for (unsigned cut = 0; cut < 8; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, sample);
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned index = 0; index < cut; ++index) { period(run, machine, before, sample); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned index = cut; index < 8; ++index) { period(run, machine, before, sample); }
    }
  }
}

unsigned addresses() {
  unsigned count = 0;
  for (unsigned target = 0; target < target_count; ++target) {
    for (const unsigned mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        for (unsigned agu = 0; agu < 8; ++agu) {
          const Case sample{.target = target, .mode = mode, .agu = agu, .y = y};
          encoding(sample);
          automatic(sample);
          ++count;
        }
      }
    }
  }
  return count;
}

unsigned value_cases() {
  unsigned count = 0;
  for (unsigned target = 0; target < target_count; ++target) {
    for (unsigned vector = 0; vector < values.size(); ++vector) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const unsigned mode : {5U, 7U}) {
          for (const bool y : {false, true}) {
            automatic({.target = target,
                       .mode = mode,
                       .agu = 7,
                       .value = values[vector],
                       .ccr = flags[vector % flags.size()],
                       .scale = scale,
                       .y = y});
            ++count;
          }
        }
      }
    }
  }
  return count;
}

void native_matrix() {
  for (unsigned target = 0; target < target_count; ++target) {
    for (const unsigned address_mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        native({.target = target, .mode = address_mode, .agu = target % 8, .y = y});
      }
    }
  }
}

void alias_snapshots() {
  for (const unsigned target : {12U, 19U, 20U, 27U, 28U, 35U}) {
    const auto agu = (target - r_begin) % 8;
    for (const unsigned address_mode : {5U, 7U}) {
      for (const bool y : {false, true}) {
        snapshots({.target = target, .mode = address_mode, .agu = agu, .y = y});
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--staged";
    if (count > 2 || (mode != "--staged" && mode != "--native")) {
      throw Fault("Usage: register-probe [--staged|--native]");
    }
    if (mode == "--native") {
      native_matrix();
      std::cout << "144 native all-destination cases passed\n";
    } else {
      const auto addressing = addresses();
      const auto values_count = value_cases();
      alias_snapshots();
      std::cout << addressing << " all-destination addresses," << values_count
                << " value/scaling cases,192 cuts/384 suffixes passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
