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

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace blofeld::test::postloop_context {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned body = boot + 6;
constexpr unsigned prefix = 4;
constexpr unsigned setup = body + prefix;
constexpr unsigned loop_body = setup + 2;
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned seed_x1 = 0x45f400;
constexpr unsigned seed_r4 = 0x64f400;
constexpr unsigned old_r4 = 0x123456;
constexpr unsigned do_prefix = 0x060080;
constexpr unsigned do_cost = 5; // FM A-1, DO #xxx,aaaa.
constexpr unsigned count_shift = 8;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned condition_mask = 0xff;
constexpr unsigned omr_long = 0x05f43a;
constexpr unsigned omr_short = 0x0589ba;
constexpr unsigned omr_direct = 0x04c5ba;
constexpr unsigned priority_omr = 0x600389;
constexpr unsigned short_omr = 0x89;
constexpr unsigned move_a_r4 = 0x21d400;
constexpr unsigned x_source = 0x10000;
constexpr unsigned p_alias = 0x1000;
constexpr unsigned canary = 0x654321;
constexpr unsigned switch_delay = 3;
constexpr unsigned bound = 256;
constexpr unsigned tail = 32;
constexpr unsigned base_instructions = 19;
constexpr unsigned last_restore_clock = 29;
enum class Form : std::uint8_t { direct, short_immediate, long_immediate };

struct Input {
  unsigned count = 3;
  unsigned size = 2;
  unsigned gap = 0;
  Form form = Form::direct;
  bool active_guard = false;

  [[nodiscard]] unsigned value() const {
    return form == Form::short_immediate ? short_omr : priority_omr;
  }

  [[nodiscard]] unsigned writer() const { return loop_body + size + gap; }

  [[nodiscard]] unsigned words() const { return form == Form::long_immediate ? 2U : 1U; }

  [[nodiscard]] unsigned move() const { return writer() + words(); }

  [[nodiscard]] unsigned second_setup() const { return move() + 4; }

  [[nodiscard]] unsigned second_body() const { return second_setup() + 2; }

  [[nodiscard]] unsigned end() const { return second_body() + 3; }
};

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram program;
  for (const auto word :
       std::array{sr_move, masked_status, seed_x1, input.value(), seed_r4, old_r4}) {
    program.word(word);
  }
  for (unsigned index = 0; index < prefix; ++index) { program.word(0); }
  program.word(do_prefix | (input.count << count_shift));
  program.word(loop_body + input.size - 1);
  for (unsigned index = 0; index < input.size; ++index) {
    program.word(input.active_guard && index == 0 ? omr_short : 0U);
  }
  for (unsigned index = 0; index < input.gap; ++index) { program.word(0); }
  switch (input.form) {
    case Form::direct: program.word(omr_direct); break;
    case Form::short_immediate: program.word(omr_short); break;
    case Form::long_immediate:
      program.word(omr_long);
      program.word(input.value());
      break;
    default: throw Fault("Undefined own OMR form");
  }
  program.word(move_a_r4);
  for (unsigned index = 0; index < switch_delay; ++index) { program.word(0); }
  program.word(do_prefix | (2U << count_shift));
  program.word(input.second_body() + 1);
  for (unsigned index = 0; index < tail; ++index) { program.word(0); }
  program.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = x_source, .value = canary});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 3);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.omr == reset_omr && before.cpu.sr == masked_status &&
              before.cpu.a == 0 && before.cpu.b == 0 && before.cpu.r[4] == old_r4 &&
              before.cpu.x == (std::uint64_t{input.value()} << word_bits) &&
              before.native_instructions == 3 && before.native_cycles == 6,
          "Three actual own native moves establish the complete literal entry context");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

struct Loop {
  DspDebugState before;
  unsigned body;
  unsigned size;
  unsigned remaining;
};

struct Oracle {
  DspDebugState expected;
  std::optional<Loop> loop;
  unsigned writer_clock = 0;
  unsigned opened = 0;
};

inline void open(Oracle &oracle, const unsigned start, const unsigned size, const unsigned count) {
  const auto before = oracle.expected;
  oracle.expected.native_cycles += do_cost;
  ++oracle.opened;
  // FM13-56/57 pushes both frame cells even when zero count annuls the body.
  oracle.expected.stack_high[1] = before.cpu.la;
  oracle.expected.stack_low[1] = before.cpu.lc;
  oracle.expected.stack_high[2] = start;
  oracle.expected.stack_low[2] = before.cpu.sr;
  if (count == 0) {
    oracle.expected.cpu.pc = start + size;
    return;
  }
  require(!oracle.loop && before.cpu.sp == 0 && before.sc == 0,
          "Only a truly completed or annulled first loop may precede the next native DO");
  oracle.loop = Loop{.before = before, .body = start, .size = size, .remaining = count};
  oracle.expected.cpu.pc = start;
  oracle.expected.cpu.la = start + size - 1;
  oracle.expected.cpu.lc = count;
  oracle.expected.cpu.sp = 2;
  oracle.expected.sc = 2;
  oracle.expected.cpu.sr |= loop_flag;
}

inline void nop(Oracle &oracle) {
  const auto pc = oracle.expected.cpu.pc;
  ++oracle.expected.cpu.pc;
  ++oracle.expected.native_cycles;
  if (!oracle.loop || pc != oracle.loop->body + oracle.loop->size - 1) { return; }
  auto &loop = *oracle.loop;
  if (--loop.remaining != 0) {
    oracle.expected.cpu.lc = loop.remaining;
    oracle.expected.cpu.pc = loop.body;
    return;
  }
  const auto before = loop.before;
  oracle.expected.cpu.la = before.cpu.la;
  oracle.expected.cpu.lc = before.cpu.lc;
  oracle.expected.cpu.sp = before.cpu.sp;
  oracle.expected.sc = before.sc;
  oracle.expected.cpu.sr =
      (oracle.expected.cpu.sr & condition_mask) | (before.cpu.sr & ~condition_mask);
  oracle.loop.reset();
}

inline void alias(DspMachine &machine, const Oracle &oracle, const unsigned clock) {
  if (oracle.writer_clock != 0 && clock >= oracle.writer_clock + switch_delay) {
    require(machine.bus().read({.address = p_alias}) == canary,
            "Three genuine subsequent periods expose the preserved physical X storage");
    return;
  }
  const auto error =
      native_access::fault([&] { static_cast<void>(machine.bus().read({.address = p_alias})); });
  require(error.contains(oracle.writer_clock != 0 ? "requires elapsed switch cycles"
                                                  : "Unmapped DSP memory"),
          "Removing completed observer metadata cannot bypass actual memory-switch latency");
}

inline void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  if (phase.transfer && phase.transfer->pc == input.move()) {
    require(phase.execute1 && phase.execute1->address == input.move(),
            "The genuine native A-to-R4 transfer publishes its zero source in its own E1");
    oracle.expected.cpu.r[4] = 0;
  }
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Actual native E2 follows independent loop/body/OMR/fallthrough ordering");
    const auto pc = oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    if (pc == setup) {
      open(oracle, loop_body, input.size, input.count);
    } else if (pc == input.second_setup()) {
      open(oracle, input.second_body(), 2, 2);
    } else if (pc == input.writer()) {
      require(!oracle.loop, "The real OMR write follows the confirmed native loop exit");
      oracle.expected.cpu.pc += input.words();
      oracle.expected.native_cycles += input.words();
      oracle.expected.cpu.omr = input.value();
      oracle.writer_clock = static_cast<unsigned>(cycle.completed_cycles);
    } else {
      nop(oracle);
    }
  }
  oracle.expected.cpu.stack_pc = oracle.expected.stack_high.at(oracle.expected.cpu.sp);
  oracle.expected.cpu.stack_sr = oracle.expected.stack_low.at(oracle.expected.cpu.sp);
  const auto actual = machine.cpu().debug_state();
  if (actual != oracle.expected) {
    const auto &expected = oracle.expected;
    std::cerr << "clock=" << cycle.completed_cycles << " count=" << input.count
              << " size=" << input.size << " gap=" << input.gap << " pc=" << hex(actual.cpu.pc, 6)
              << "/" << hex(expected.cpu.pc, 6) << " sr=" << hex(actual.cpu.sr, 6) << "/"
              << hex(expected.cpu.sr, 6) << " omr=" << hex(actual.cpu.omr, 6) << "/"
              << hex(expected.cpu.omr, 6) << " la=" << hex(actual.cpu.la, 6) << "/"
              << hex(expected.cpu.la, 6) << " lc=" << hex(actual.cpu.lc, 6) << "/"
              << hex(expected.cpu.lc, 6) << " sp=" << actual.cpu.sp << "/" << expected.cpu.sp
              << " sc=" << actual.sc << "/" << expected.sc << " native=" << actual.native_cycles
              << "/" << expected.native_cycles << " instructions=" << actual.native_instructions
              << "/" << expected.native_instructions << " stack1=" << hex(actual.stack_high[1], 6)
              << ":" << hex(actual.stack_low[1], 6) << "/" << hex(expected.stack_high[1], 6) << ":"
              << hex(expected.stack_low[1], 6) << " stack2=" << hex(actual.stack_high[2], 6) << ":"
              << hex(actual.stack_low[2], 6) << "/" << hex(expected.stack_high[2], 6) << ":"
              << hex(expected.stack_low[2], 6) << '\n';
  }
  require(machine.cpu().debug_state() == oracle.expected &&
              machine.reference_fetch().omr == oracle.expected.cpu.omr,
          "Full CPU, literal DO frames, flags, costs and logical OMR match every period");
  alias(machine, oracle, static_cast<unsigned>(cycle.completed_cycles));
}

inline void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (oracle.expected.cpu.pc != input.end() && run.completed_cycles() < bound) {
    step(run, machine, input, oracle);
  }
  require(oracle.expected.cpu.pc == input.end() && !oracle.loop && oracle.opened == 2 &&
              oracle.writer_clock != 0 &&
              oracle.expected.native_instructions ==
                  base_instructions + input.count * input.size + input.gap,
          "Both actual loop lifetimes, real OMR and every literal successor finish natively");
}

inline void one_case(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input), .loop = std::nullopt};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

inline void restores(const Input input) {
  for (unsigned cut = 15; cut <= last_restore_clock; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input), .loop = std::nullopt};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix_state = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix_state;
      finish(run, machine, input, oracle);
    }
  }
}

inline void active_guard() {
  DspMachine machine;
  const Input input{.size = 4, .active_guard = true};
  static_cast<void>(prepare(machine, input));
  DspFetchRun run(contract(), machine);
  const auto error = native_access::fault([&] { pipeline::clocks(run, machine, bound); });
  const auto state = machine.cpu().state();
  require(error.contains("Unqualified native OMR context") && error.contains("pc=") &&
              error.contains("omr=") && run.faulted() && state.omr == reset_omr &&
              state.r[4] == old_r4 && (state.sr & loop_flag) != 0,
          "An active loop still refuses OMR before its architectural result is published");
}
}
