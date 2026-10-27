// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>

namespace blofeld::test::scalar_store {
inline constexpr unsigned entry = 0x100, data = 0x400, canary = 0x654321;
inline constexpr unsigned ordinary_omr = 0x300, old_lc = 0x33, absolute_jump = 0x0af080;
// DSP56300FM13-119/123: A/B fields 14/15, W=0, EA=1, no parallel ALU.
inline constexpr unsigned x_a_store = 0x564000, b_bit = 0x10000, y_bit = 0x80000;
inline constexpr unsigned mode_shift = 11, register_shift = 8;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

struct Case {
  std::uint64_t value = 0;
  unsigned scale = 0;
  unsigned flags = 0;
  unsigned mode = 3;
  unsigned reg = 0;
  bool b = false;
  bool y = false;
};

inline unsigned opcode(const Case sample) {
  return x_a_store | (sample.b ? b_bit : 0U) | (sample.y ? y_bit : 0U) |
         (sample.mode << mode_shift) | (sample.reg << register_shift);
}

inline unsigned pointer(const unsigned mode) {
  switch (mode) {
    case 0: return data - 3;
    case 1: return data + 3;
    case 2: return data - 1;
    case 3: return data + 1;
    case 4: return data;
    default: throw Fault("Undefined own scalar-store address mode");
  }
}

inline void prepare(DspMachine &machine,
                    const Case sample,
                    const std::optional<unsigned> replacement = std::nullopt) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("lc", old_lc);
  program.immediate("r" + std::to_string(sample.reg), data);
  program.immediate("n" + std::to_string(sample.reg), 3);
  const std::string reg = sample.b ? "b" : "a";
  for (unsigned field = 0; field < 3; ++field) {
    const auto value = (sample.value >> (field * word_bits)) & (field == 2 ? 0xffU : word_mask);
    program.immediate(reg + std::to_string(field), value);
  }
  program.immediate("sr", masked_status | sample.scale | sample.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned address = entry; address < entry + 16; ++address) {
    machine.bus().write({.address = address, .value = 0});
  }
  machine.bus().write({.address = entry, .value = replacement.value_or(opcode(sample))});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
}

inline DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = execution;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

struct Result {
  unsigned value;
  unsigned flags;
};

inline Result expected(const Case sample) {
  // FM3.5,5-13: signed56, scaling, signed24 limiter and source growth bits.
  constexpr std::int64_t sign56 = 0x80000000000000;
  constexpr std::int64_t span56 = 0x100000000000000;
  constexpr std::int64_t maximum = 0x7fffff;
  constexpr std::int64_t minimum = -0x800000;
  auto value = static_cast<std::int64_t>(sample.value);
  if (value >= sign56) { value -= span56; }
  constexpr unsigned normal_growth = 46;
  unsigned growth = normal_growth;
  if (sample.scale == scale_up) {
    value *= 2;
    --growth;
  } else if (sample.scale == scale_down) {
    value >>= 1U;
    ++growth;
  }
  value >>= word_bits;
  auto flags = sample.flags;
  if (((sample.value >> growth) & 1U) != ((sample.value >> (growth - 1)) & 1U)) { flags |= sticky; }
  if (value > maximum) {
    value = maximum;
    flags |= limit;
  } else if (value < minimum) {
    value = minimum;
    flags |= limit;
  }
  return {.value = static_cast<unsigned>(static_cast<std::uint64_t>(value) & word_mask),
          .flags = flags};
}

inline void cells(DspMachine &machine, const Case sample, const unsigned wanted) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      const bool destination = address == data && (space == DspSpace::y) == sample.y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (destination ? wanted : canary),
              "Scalar store must preserve other space and neighboring cells");
    }
  }
}

inline void verify(DspMachine &machine,
                   const DspDebugState &before,
                   const Case sample,
                   const DspFetchCycleResult &cycle,
                   const unsigned period) {
  const auto value = expected(sample);
  auto wanted = before;
  if (period >= 5) { wanted.cpu.r.at(sample.reg) = pointer(sample.mode); }
  if (period == 7) {
    wanted.cpu.pc = entry + 1;
    wanted.cpu.sr = masked_status | sample.scale | value.flags;
    ++wanted.native_instructions;
    ++wanted.native_cycles;
  }
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "period=" << period << " opcode=" << hex(opcode(sample), 6)
              << " input=" << hex(sample.value, 14) << " scale=" << sample.scale
              << " sr actual/expected=" << hex(actual.cpu.sr, 6) << '/' << hex(wanted.cpu.sr, 6)
              << '\n';
  }
  require(actual == wanted, "Scalar store publishes only genuine AG2/E2 state");
  cells(machine, sample, period >= 6 ? value.value : canary);
  require(cycle.pipeline && cycle.pipeline->memory.has_value() == (period == 6),
          "Scalar store has exactly one actual native E1 bus operation");
}

inline void one(const Case sample, const bool rejection) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned period = 1; period <= (rejection ? 2U : 7U); ++period) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    verify(machine, before, sample, cycle, period);
  }
  if (!rejection) { return; }
  const auto error = checkpoint::rejects(
      [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
      "Expected current scalar-store decode refusal");
  require(error.contains("automatic DSP decode") && error.contains("pc=0x000100") &&
              error.contains("omr=0x000300") && run.faulted() && run.completed_cycles() == 2 &&
              machine.cpu().debug_state() == before,
          "Baseline must expose unsupported decode before AG2/native effects");
  cells(machine, sample, canary);
  std::cout << "Baseline refusal opcode=" << hex(opcode(sample), 6) << ": " << error << '\n';
}

inline void restore_case(const Case sample, const unsigned cut) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned period = 1; period <= cut; ++period) {
    verify(machine, before, sample, run.advance_cycle(machine, fetch_clock::input()), period);
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    for (unsigned period = cut + 1; period <= 7; ++period) {
      verify(machine, before, sample, run.advance_cycle(machine, fetch_clock::input()), period);
    }
  }
}

inline void restores() {
  constexpr std::uint64_t clipped_input = 0x00800000000001;
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        const Case sample{.value = clipped_input, .scale = scale, .b = b, .y = y};
        for (unsigned cut = 0; cut < 7; ++cut) { restore_case(sample, cut); }
      }
    }
  }
}

inline void guards() {
  // Full loads, partial sources, parallel ALU, and modes5..7 stay unqualified.
  constexpr std::array refused{0x56d800U,
                               0x5fd800U,
                               0x505800U,
                               0x525800U,
                               0x545800U,
                               0x565840U,
                               0x566800U,
                               0x567000U,
                               0x567800U};
  constexpr unsigned reserved_scale = 0xc00;
  for (const auto word : refused) {
    DspMachine machine;
    prepare(machine, {}, word);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    for (unsigned period = 1; period <= 2; ++period) {
      static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    }
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unqualified scalar form must refuse");
    require(error.contains("automatic DSP decode") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000300") && run.faulted() && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before,
            "Unqualified forms refuse before architectural and memory effects");
    cells(machine, {}, canary);
  }
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      const Case sample{.scale = reserved_scale, .b = b, .y = y};
      DspMachine machine;
      prepare(machine, sample);
      const auto before = machine.cpu().debug_state();
      DspFetchRun run(contract(), machine);
      for (unsigned period = 1; period <= 2; ++period) {
        static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
      }
      const auto saved = run.checkpoint(machine);
      for (unsigned replay = 0; replay < 2; ++replay) {
        run.restore(saved, machine);
        const auto error = checkpoint::rejects(
            [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
            "Reserved scalar-store scaling must refuse");
        require(error.contains("accumulator MOVE scaling read P:") && error.contains("sr=") &&
                    error.contains("pc=0x000100") && error.contains("omr=0x000300") &&
                    run.faulted() && run.completed_cycles() == 2 &&
                    machine.cpu().debug_state() == before,
                "Reserved scaling refuses before AG2/memory in restored continuations");
        cells(machine, sample, canary);
      }
    }
  }
}
}
