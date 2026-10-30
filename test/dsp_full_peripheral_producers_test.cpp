// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned seed_count = 7, body = boot + 14, padding = 24;
constexpr unsigned compare = 0xffff8d, canary = 0x654321;
constexpr unsigned b_field = 0x100, source_shift = 24, extension_shift = 48;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned once_a_store = 0x08ce3c, compare_a_store = 0x04ce8d;
constexpr std::uint64_t positive = 0x00123456abcdef, negative_source = 0xffbfffffabcdef;
constexpr std::uint64_t carry_source = 0x003fffffffffff, carry_result = 0x00400000000000;
constexpr std::uint64_t fractional_one = 0x010000000000;
// FM13-118/122/Table12-14: short immediate1 to A0,A1,A2,A; FM13-77 INC A.
constexpr std::array producer_words{0x280100U, 0x2c0100U, 0x2a0100U, 0x2e0100U, 0x000008U};
enum class Kind : std::uint8_t { low, middle, extension, full, increment };
constexpr std::array move_kinds{Kind::low, Kind::middle, Kind::extension, Kind::full};

struct Input {
  std::uint64_t value = positive;
  unsigned scale = 0, flags = 0, gap = 0;
  bool b = false, timer = false, same = false;
  Kind kind = Kind::low;
};

bool writer_b(const Input input) {
  return input.same ? input.b : !input.b;
}

unsigned producer_opcode(const Input input) {
  constexpr unsigned move_b = 0x10000;
  const unsigned field = input.kind == Kind::increment ? 1U : move_b;
  return producer_words.at(static_cast<unsigned>(input.kind)) | (writer_b(input) ? field : 0U);
}

std::uint64_t produced(const Input input) {
  if (input.kind == Kind::increment) { return carry_result; }
  if (input.kind == Kind::full) { return fractional_one; }
  const unsigned shift = static_cast<unsigned>(input.kind) * word_bits;
  const std::uint64_t mask = input.kind == Kind::extension ? byte_mask : word_mask;
  return (input.value & ~(mask << shift)) | (std::uint64_t{1} << shift);
}

unsigned opcode(const Input input) {
  // FM13-136/Table12-13: full A/B fields14/15; pp=60 or split qq=13.
  return (input.timer ? compare_a_store : once_a_store) | (input.b ? b_field : 0U);
}

unsigned store_pc(const Input input) {
  return body + 2 + input.gap;
}

unsigned stall(const Input input) {
  return input.same && input.gap == 0 ? 1U : 0U;
}

unsigned e1(const Input input) {
  return 8 + input.gap + stall(input);
}

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  for (const auto *name : {"a", "b"}) {
    code.immediate(std::string(name) + '0', input.value & word_mask);
    code.immediate(std::string(name) + '1', (input.value >> source_shift) & word_mask);
    code.immediate(std::string(name) + '2', (input.value >> extension_shift) & byte_mask);
  }
  code.immediate("sr", masked_status | input.scale | input.flags);
  code.word(0);
  code.word(producer_opcode(input));
  for (unsigned gap = 0; gap < input.gap; ++gap) { code.word(0); }
  code.word(opcode(input));
  for (unsigned i = 0; i < padding; ++i) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = compare, .value = canary});
  checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == input.value && before.cpu.b == input.value &&
              before.cpu.sr == (masked_status | input.scale | input.flags),
          "Native immediate seeds establish the full independently selected accumulator state");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  return before;
}

struct Oracle {
  DspDebugState state;
  unsigned stores = 0, stalls = 0;
};

void check_port(DspMachine &machine,
                const Input input,
                const unsigned period,
                const unsigned value) {
  const auto latch = machine.bus().once_latch();
  if (input.timer) {
    require(!latch && machine.bus().read({.space = DspSpace::x, .address = compare}) ==
                          (period >= e1(input) ? value : canary),
            "The real low-X timer compare retains the independently limited full-source value");
  } else {
    require(period >= e1(input) ? latch == value : !latch,
            "The real OnCE latch retains the independently limited full-source value at E1");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  const auto source = input.same ? produced(input) : input.value;
  const unsigned flags = input.kind == Kind::increment ? 0U : input.flags;
  const auto stored =
      scalar_store::expected({.value = source, .scale = input.scale, .flags = flags});
  const unsigned index = period >= 7 ? period - 7 : 0;
  const unsigned store_index = store_pc(input) - body;
  const bool retired = period >= 7 && (stall(input) == 0 || index != store_index);
  require(phase.retirement_published == retired,
          "Literal one-word retirements include exactly one source interlock");
  const unsigned written_period = input.kind == Kind::increment ? 8U : 7U;
  if (period == written_period) {
    (writer_b(input) ? oracle.state.cpu.b : oracle.state.cpu.a) = produced(input);
  }
  if (phase.memory_source_stall) {
    ++oracle.stalls;
    require(stall(input) == 1 && period == e1(input) - 1 && !phase.memory,
            "Only an adjacent same-accumulator producer stalls the actual E1 source");
  }
  require(phase.memory.has_value() == (period == e1(input)),
          "The native peripheral transaction happens once at the independently scheduled E1");
  if (phase.memory) { ++oracle.stores; }
  check_port(machine, input, period, stored.value);
  if (retired) {
    require(phase.execute2 && phase.execute2->address == oracle.state.cpu.pc,
            "Native E2 preserves exact instruction ownership and ordering");
    if (input.kind == Kind::increment && oracle.state.cpu.pc == body + 1) {
      // FM13-77/5.3: positive normalized400000:000000, no carry/overflow/extension/zero.
      oracle.state.cpu.sr = masked_status;
    }
    if (oracle.state.cpu.pc == store_pc(input)) {
      oracle.state.cpu.sr = masked_status | input.scale | stored.flags;
    }
    ++oracle.state.cpu.pc;
    ++oracle.state.native_instructions;
    ++oracle.state.native_cycles;
  }
  if (machine.cpu().debug_state() != oracle.state) {
    std::cerr << "period=" << period << " opcode=" << hex(opcode(input), 6)
              << " value=" << hex(input.value, 14) << " scale=" << input.scale
              << " actualSR=" << hex(machine.cpu().state().sr, 6)
              << " expectedSR=" << hex(oracle.state.cpu.sr, 6) << '\n';
  }
  require(machine.cpu().debug_state() == oracle.state,
          "Every period matches complete independent CPU/flags/physical-stack/counter state");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < e1(input) + 3) { step(run, machine, input, oracle); }
  require(oracle.stores == 1 && oracle.stalls == stall(input),
          "The complete stream has one native write and its exact source-stall count");
}

void one(const Input input) {
  DspMachine machine;
  Oracle oracle{.state = prepare(machine, input)};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void restores(const Input input) {
  for (unsigned cut = 6; cut <= 10; ++cut) {
    DspMachine machine;
    Oracle oracle{.state = prepare(machine, input)};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix;
      finish(run, machine, input, oracle);
    }
    DspMachine clone;
    DspFetchRun continuation(saved, clone);
    oracle = prefix;
    finish(continuation, clone, input, oracle);
  }
}

unsigned streams = 0;

void visit(const Input input) {
  one(input);
  ++streams;
  if (input.gap == 0 && input.flags == 0 &&
      (input.value == positive || input.kind == Kind::increment)) {
    restores(input);
  }
}

void move_cases(const bool b, const bool timer) {
  for (const auto kind : move_kinds) {
    for (const auto value : {positive, negative_source}) {
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        for (const unsigned flags : {0U, byte_mask}) {
          for (const bool same : {false, true}) {
            for (unsigned gap = 0; gap <= 2; ++gap) {
              visit({.value = value,
                     .scale = scale,
                     .flags = flags,
                     .gap = gap,
                     .b = b,
                     .timer = timer,
                     .same = same,
                     .kind = kind});
            }
          }
        }
      }
    }
  }
}

void increment_cases(const bool b, const bool timer) {
  for (const bool same : {false, true}) {
    for (unsigned gap = 0; gap <= 2; ++gap) {
      visit({.value = carry_source,
             .gap = gap,
             .b = b,
             .timer = timer,
             .same = same,
             .kind = Kind::increment});
    }
  }
}
}

int main() {
  try {
    for (const bool b : {false, true}) {
      for (const bool timer : {false, true}) {
        move_cases(b, timer);
        increment_cases(b, timer);
      }
    }
    std::cout << "Full MOVEP producer streams=" << streams
              << ";1040suffixes;520clones;A0/A1/A2/full/INC and both sources\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
