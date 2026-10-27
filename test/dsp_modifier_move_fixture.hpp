// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
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
#include <iostream>
#include <optional>
#include <string>

namespace blofeld::test::modifier_move {
inline constexpr unsigned entry = 0x100, ordinary_omr = 0x300, absolute_jump = 0x0af080;
inline constexpr unsigned modifier_seed = 0x070100, extension_mask = 0xff;
// FM13-131/12-20: W=1,eeeeee=100sss,ddddd=00ddd or W=0,eeeeee=00111b,ddddd=00sss.
inline constexpr unsigned m_to_m = 0x04e0a0, m_to_a = 0x044ea0, register_shift = 8;
inline constexpr unsigned accumulator_begin = 8, destination_count = 10, register_count = 8;
inline constexpr unsigned sign24 = 0x800000;
inline constexpr std::uint64_t negative_extension = 0xff000000000000;
inline constexpr std::uint64_t initial_a = 0x34567890123456, initial_b = 0x89abcdefedcba9;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

struct Case {
  unsigned source = 0;
  unsigned destination = 1;
  unsigned value = 0;
  unsigned scale = 0;
  unsigned flags = 0;
};

inline unsigned opcode(const Case sample) {
  if (sample.destination < accumulator_begin) {
    return m_to_m | (sample.source << register_shift) | sample.destination;
  }
  return m_to_a | ((sample.destination - accumulator_begin) << register_shift) | sample.source;
}

inline std::uint64_t expanded(const unsigned value) {
  return (std::uint64_t{value} << word_bits) | ((value & sign24) != 0 ? negative_extension : 0U);
}

inline void destination(DspState &state, const Case sample) {
  if (sample.destination < accumulator_begin) {
    state.m.at(sample.destination) = sample.value;
  } else if (sample.destination == accumulator_begin) {
    state.a = expanded(sample.value);
  } else {
    state.b = expanded(sample.value);
  }
}

inline void prepare(DspMachine &machine,
                    const Case sample,
                    const std::optional<unsigned> replacement = std::nullopt) {
  require(sample.source < register_count && sample.destination < destination_count,
          "Own modifier transfer source/destination must fit literal fields");
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  for (unsigned index = 0; index < register_count; ++index) {
    program.immediate("m" + std::to_string(index),
                      index == sample.source ? sample.value : modifier_seed + index);
  }
  for (const bool b : {false, true}) {
    const auto value = b ? initial_b : initial_a;
    const std::string name = b ? "b" : "a";
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(name + std::to_string(part),
                        (value >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("sr", masked_status | sample.scale | sample.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned cursor = entry; cursor < entry + 32; ++cursor) {
    machine.bus().write({.address = cursor, .value = 0});
  }
  machine.bus().write({.address = entry, .value = replacement.value_or(opcode(sample))});
  const auto before = machine.cpu().state();
  require(before.pc == entry && before.a == initial_a && before.b == initial_b &&
              before.sr == (masked_status | sample.scale | sample.flags),
          "Independent literal seeds establish both accumulators and every input CCR/mode bit");
  for (unsigned index = 0; index < register_count; ++index) {
    require(before.m.at(index) == (index == sample.source ? sample.value : modifier_seed + index),
            "Each raw modifier input must match its independently declared seed");
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
}

inline DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = execution;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline void verify(const DspMachine &machine,
                   const DspDebugState &before,
                   const Case sample,
                   const DspFetchCycleResult &cycle,
                   const unsigned period) {
  auto wanted = before;
  if (period >= 6) { destination(wanted.cpu, sample); }
  if (period == 7) {
    ++wanted.cpu.pc;
    ++wanted.native_instructions;
    ++wanted.native_cycles;
  }
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "period=" << period << " opcode=" << hex(opcode(sample), 6)
              << " value=" << hex(sample.value, 6) << " scale=" << hex(sample.scale, 3)
              << " source=" << sample.source << " destination=" << sample.destination
              << " a=" << hex(actual.cpu.a, 14) << '/' << hex(wanted.cpu.a, 14)
              << " b=" << hex(actual.cpu.b, 14) << '/' << hex(wanted.cpu.b, 14)
              << " sr=" << hex(actual.cpu.sr, 6) << '/' << hex(wanted.cpu.sr, 6) << '\n';
  }
  require(actual == wanted,
          "Raw modifiers publish native E1 fields/E2 retirement; sign extension bypasses scaling "
          "and S/L");
  require(cycle.pipeline && cycle.pipeline->transfer.has_value() == (period == 6) &&
              !cycle.pipeline->memory && !cycle.pipeline->memory_source_stall,
          "A standalone modifier transfer has one real register E1 and no fabricated memory/stall");
}

inline void one(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  for (unsigned period = 1; period <= 7; ++period) {
    verify(machine, before, sample, run.advance_cycle(machine, fetch_clock::input()), period);
  }
}

inline void refusal(const Case sample, const std::optional<unsigned> replacement = std::nullopt) {
  DspMachine machine;
  prepare(machine, sample, replacement);
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
        "Unqualified modifier-source form unexpectedly entered the native pipeline");
    require(
        error.contains("Unqualified") && error.contains("pc=0x000100") &&
            error.contains("omr=0x000300") && run.faulted() && run.completed_cycles() == 2 &&
            machine.cpu().debug_state() == before,
        "Unknown control forms refuse with context before native effects in both continuations");
  }
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
  constexpr std::array examples{Case{.source = 0, .destination = 7, .value = word_mask},
                                Case{.source = 7, .destination = 0, .value = 7},
                                Case{.source = 1, .destination = 8, .value = word_mask},
                                Case{.source = 6, .destination = 9, .value = sign24}};
  for (auto sample : examples) {
    for (const unsigned scale : {0U, scale_down, scale_up}) {
      sample.scale = scale;
      for (unsigned cut = 0; cut < 7; ++cut) { restore_case(sample, cut); }
    }
  }
}

inline void guards() {
  // M0->X0/Y0/R0/N0/A0/B1/SR/SP/SSH/SSL/LC, N0->M1 and undocumented bit6.
  constexpr std::array words{0x0444a0U,
                             0x0446a0U,
                             0x0450a0U,
                             0x0458a0U,
                             0x0448a0U,
                             0x044da0U,
                             0x0479a0U,
                             0x047ba0U,
                             0x047ca0U,
                             0x047da0U,
                             0x047fa0U,
                             0x04d8a1U,
                             0x04e0e1U,
                             0x044ee1U};
  for (const auto word : words) { refusal({}, word); }
}
}
