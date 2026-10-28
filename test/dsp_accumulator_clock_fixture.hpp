// SPDX-License-Identifier: GPL-3.0-only
#pragma once
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
#include "native_access_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace blofeld::test::accumulator_clock {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned r4_field = 20;
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned move_prefix = 0x200000;
constexpr unsigned body = boot + 14;
constexpr unsigned writer = body + 1;
constexpr unsigned first_e2 = 7;
constexpr unsigned writer_e2 = 8;
constexpr unsigned last = 11;
constexpr unsigned tail = 32;
constexpr unsigned status_flags = 0xc0033f;
constexpr unsigned limit_flag = 0x40;
constexpr unsigned growth_flag = 0x80;
constexpr std::uint64_t source_sign = std::uint64_t{1} << 55U;
constexpr std::int64_t source_span = std::int64_t{1} << 56U;
constexpr std::int64_t positive_limit = 0x7fffff;
constexpr std::int64_t negative_limit = -0x800000;
constexpr std::array seed_opcodes{0x50f400U, 0x54f400U, 0x52f400U, 0x51f400U, 0x55f400U, 0x53f400U};
constexpr std::array destinations{4U, 5U, 6U, 7U, 16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U};
constexpr std::array<std::uint64_t, 8> values{0,
                                              0x123456abcdef,
                                              0x7fffffffffff,
                                              0x800000000000,
                                              0xff800000000000,
                                              0xff7fffffffffff,
                                              0x01000000000000,
                                              0xfeffffff123456};

enum class Source : std::uint8_t { a, b };

struct Input {
  Source source = Source::a;
  unsigned destination = r4_field;
  unsigned scale = 0;
  std::uint64_t value = 0;
  bool sticky = false;
};

struct Transfer {
  unsigned value;
  unsigned flags;
};

inline unsigned opcode(const Input input) {
  const unsigned source = input.source == Source::a ? 14U : 15U;
  return move_prefix | (source << 13U) | (input.destination << 8U);
}

inline Transfer reference(const Input input) {
  auto signed_value = static_cast<std::int64_t>(input.value);
  if ((input.value & source_sign) != 0) { signed_value -= source_span; }
  auto scaled = signed_value;
  if (input.scale == 1) { scaled >>= 1U; }
  if (input.scale == 2) { scaled *= 2; }
  const auto word = scaled >> word_bits;
  const auto clipped = std::clamp(word, negative_limit, positive_limit);
  constexpr std::array growth_bit{46U, 47U, 45U};
  const auto bit = growth_bit.at(input.scale);
  const bool growth = (((input.value >> bit) ^ (input.value >> (bit - 1))) & 1U) != 0;
  return {.value = static_cast<unsigned>(clipped) & word_mask,
          .flags = (word != clipped ? limit_flag : 0U) | (growth ? growth_flag : 0U)};
}

inline DspDebugState prepare(DspMachine &machine,
                             const Input input,
                             const std::optional<unsigned> replacement = std::nullopt) {
  DspProgram code;
  for (unsigned index = 0; index < seed_opcodes.size(); ++index) {
    const bool active = (index < 3) == (input.source == Source::a);
    const auto part = index % 3;
    const auto mask = part == 2 ? 0xffU : word_mask;
    code.word(seed_opcodes.at(index));
    code.word(active ? static_cast<unsigned>((input.value >> (part * word_bits)) & mask) : 0U);
  }
  const auto status = status_flags | (input.scale << 10U) | (input.sticky ? growth_flag : 0U);
  code.word(sr_move);
  code.word(status);
  code.word(0);
  code.word(replacement.value_or(opcode(input)));
  for (unsigned index = 0; index < tail; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 7);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.sr == status &&
              before.cpu.a == (input.source == Source::a ? input.value : 0U) &&
              before.cpu.b == (input.source == Source::b ? input.value : 0U) &&
              before.native_instructions == 7 && before.native_cycles == 14,
          "Independent literal native seeds establish full56-bit sources and exact flags/costs");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

inline DspFetchRunInputs contract() {
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  return input;
}

inline void destination(DspState &state, const Input input, const unsigned value) {
  if (input.destination >= 16) {
    state.r.at(input.destination - 16) = value;
    return;
  }
  auto &pair = input.destination < 6 ? state.x : state.y;
  const auto shift = (input.destination & 1U) * word_bits;
  pair = (pair & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
}

struct Oracle {
  DspDebugState expected;
  unsigned retired = 0;
};

inline void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto transfer = reference(input);
  if (cycle.completed_cycles == first_e2) {
    require(phase.execute1 && phase.execute1->address == writer && phase.transfer,
            "The actual one-word source transfer publishes its real native destination in E1");
    destination(oracle.expected.cpu, input, transfer.value);
  }
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Actual native retirement preserves the declared sequential instruction order");
    if (oracle.retired == 1) {
      require(cycle.completed_cycles == writer_e2,
              "Independent one-unit register MOVE does not insert artificial waiting periods");
      oracle.expected.cpu.sr |= transfer.flags;
    }
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  const auto actual = machine.cpu().debug_state();
  if (actual != oracle.expected) {
    const auto &expected = oracle.expected;
    std::cerr << "cycle=" << cycle.completed_cycles
              << " source=" << (input.source == Source::a ? "a" : "b")
              << " destination=" << input.destination << " scale=" << input.scale
              << " input=" << hex(input.value, 14) << " initialS=" << input.sticky
              << " pc=" << hex(actual.cpu.pc, 6) << "/" << hex(expected.cpu.pc, 6)
              << " sr=" << hex(actual.cpu.sr, 6) << "/" << hex(expected.cpu.sr, 6)
              << " x=" << hex(actual.cpu.x, 12) << "/" << hex(expected.cpu.x, 12)
              << " y=" << hex(actual.cpu.y, 12) << "/" << hex(expected.cpu.y, 12)
              << " a=" << hex(actual.cpu.a, 14) << "/" << hex(expected.cpu.a, 14)
              << " instructions=" << actual.native_instructions << "/"
              << expected.native_instructions << " cycles=" << actual.native_cycles << "/"
              << expected.native_cycles << '\n';
  }
  require(actual == oracle.expected,
          "Full native CPU/frame/count state follows independent signed scaling, clipping and S/L");
}

inline void one_case(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input), .retired = 0};
  DspFetchRun run(contract(), machine);
  while (run.completed_cycles() < last) { step(run, machine, input, oracle); }
  require(oracle.retired == 5 && run.elapsed_ticks() == last * pipeline::period,
          "NOP, real accumulator MOVE and three successors retain independent nominal costs");
}

inline void restores() {
  for (const auto source : {Source::a, Source::b}) {
    for (unsigned scale = 0; scale < 3; ++scale) {
      const Input input{.source = source, .scale = scale, .value = values[2]};
      for (unsigned cut = 4; cut <= 10; ++cut) {
        DspMachine machine;
        Oracle oracle{.expected = prepare(machine, input), .retired = 0};
        DspFetchRun run(contract(), machine);
        while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
        const auto saved = run.checkpoint(machine);
        const auto prefix = oracle;
        for (unsigned replay = 0; replay < 2; ++replay) {
          run.restore(saved, machine);
          oracle = prefix;
          while (run.completed_cycles() < last) { step(run, machine, input, oracle); }
          require(oracle.retired == 5, "Joint restoration preserves actual E1 fields and E2 flags");
        }
      }
    }
  }
}

inline void reserved_scaling() {
  DspMachine machine;
  const auto before = prepare(machine, {.scale = 3});
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 6);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("accumulator MOVE scaling read P:") && error.contains("pc=") &&
                error.contains("sr=") && error.contains("omr=") && run.faulted() &&
                run.completed_cycles() == 6 && machine.cpu().debug_state() == before,
            "Reserved scaling refuses before native E1/E2 effects in both trusted continuations");
  }
}

inline void guards() {
  constexpr std::array unqualified{
      move_prefix | (8U << 13U) | (4U << 8U),   // A0 source remains separately unqualified.
      move_prefix | (14U << 13U) | (24U << 8U), // N0 destination needs separate AGU qualification.
      move_prefix | (14U << 13U) | (20U << 8U) | 0x40U}; // Parallel source is not yet qualified.
  for (const auto word : unqualified) {
    DspMachine machine;
    const auto before = prepare(machine, {}, word);
    DspFetchRun run(contract(), machine);
    pipeline::clocks(run, machine, 3);
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("automatic DSP decode") && error.contains("pc=") &&
                error.contains("omr=") && machine.cpu().debug_state() == before &&
                run.completed_cycles() == 3 && run.faulted(),
            "Unsupported source/destination/parallel forms fail before CPU effects");
  }
}
}
