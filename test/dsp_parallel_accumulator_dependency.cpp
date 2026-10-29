// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_accumulator_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld::test::parallel_accumulator {
namespace {
constexpr unsigned bound = 64;
// FM13-7/118: own ordinary ADD and raw middle-field register MOVE encodings.
constexpr unsigned add_a_b = 0x200018, add_b_a = 0x200010;
constexpr unsigned a1_to_x0 = 0x218400, b1_to_x0 = 0x21a400;

enum class Kind : std::uint8_t {
  source_bus,
  sum_bus,
  alu_same,
  alu_reverse,
  memory_twice,
  incoming_source,
  incoming_sum
};

struct Case {
  Input input;
  Kind kind = Kind::source_bus;
  unsigned gap = 0;
};

bool incoming(const Case sample) {
  return sample.kind == Kind::incoming_source || sample.kind == Kind::incoming_sum;
}

unsigned other_instruction(const Case sample) {
  const bool b = sample.input.b;
  switch (sample.kind) {
    case Kind::source_bus: return b ? b1_to_x0 : a1_to_x0;
    case Kind::sum_bus: return b ? a1_to_x0 : b1_to_x0;
    case Kind::alu_reverse:
    case Kind::incoming_source: return b ? add_a_b : add_b_a;
    case Kind::alu_same:
    case Kind::incoming_sum: return b ? add_b_a : add_a_b;
    case Kind::memory_twice: return opcode(sample.input);
    default: throw Fault("Undefined own ADD/load dependency");
  }
}

DspDebugState prepare_case(DspMachine &machine, const Case sample, const bool unknown = false) {
  const auto critical = opcode(sample.input);
  const auto other = other_instruction(sample);
  std::vector<std::uint32_t> code{incoming(sample) ? other : critical};
  for (unsigned quiet = 0; quiet < sample.gap; ++quiet) { code.push_back(0); }
  code.push_back(incoming(sample) ? critical : other);
  const auto before = prepare(machine, sample.input, code, unknown);
  if (sample.kind == Kind::memory_twice) {
    machine.bus().write({.space = sample.input.y ? DspSpace::y : DspSpace::x,
                         .address = pointer + 1,
                         .value = second_word});
  }
  return before;
}

DspFetchRun clock(DspMachine &machine) {
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  return {contract, machine};
}

void arithmetic(DspState &state, const bool b) {
  const auto result = addition({.a = state.a, .b = state.b}, state.sr);
  (b ? state.b : state.a) = result.value;
  state.sr = result.status;
}

DspDebugState expected(DspDebugState before, const Case sample) {
  auto &state = before.cpu;
  const bool b = sample.input.b;
  if (incoming(sample)) { arithmetic(state, sample.kind == Kind::incoming_source ? b : !b); }
  arithmetic(state, !b);
  (b ? state.b : state.a) = loaded(first_word);
  if (sample.kind == Kind::source_bus || sample.kind == Kind::sum_bus) {
    const bool source_b = sample.kind == Kind::source_bus ? b : !b;
    state.x = (state.x & ~std::uint64_t{word_mask}) |
              (((source_b ? state.b : state.a) >> word_bits) & word_mask);
  } else if (!incoming(sample)) {
    arithmetic(state, sample.kind == Kind::alu_reverse ? b : !b);
    if (sample.kind == Kind::memory_twice) { (b ? state.b : state.a) = loaded(second_word); }
  }
  state.pc += 2 + sample.gap;
  state.r.at(sample.input.reg) = pointer + (sample.kind == Kind::memory_twice ? 2U : 1U);
  before.native_instructions += 2 + sample.gap;
  before.native_cycles += 2 + sample.gap;
  return before;
}

void step(DspFetchRun &run, DspMachine &machine, const Case sample, unsigned &reads) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  if (!cycle.bus.memory) { return; }
  const auto &memory = *cycle.bus.memory;
  require(memory.data.kind == DspNativeAccessKind::read && !memory.additional_data &&
              memory.data.target.space == (sample.input.y ? DspSpace::y : DspSpace::x) &&
              memory.data.target.address == pointer + reads &&
              memory.data.value == (reads == 0 ? first_word : second_word),
          "Only the actual selected once-only memory response feeds the parallel load");
  ++reads;
}

void finish(DspFetchRun &run,
            DspMachine &machine,
            const Case sample,
            const DspDebugState &before,
            unsigned &reads) {
  const auto after = expected(before, sample);
  while (machine.cpu().state().pc != after.cpu.pc) {
    require(run.completed_cycles() < bound, "Own dependency window exceeded its bound");
    step(run, machine, sample, reads);
  }
  const bool bus = sample.kind == Kind::source_bus || sample.kind == Kind::sum_bus;
  const unsigned clocks = 8 + sample.gap + (bus && sample.gap == 0 ? 1U : 0U);
  require(machine.cpu().debug_state() == after && run.completed_cycles() == clocks &&
              reads == (sample.kind == Kind::memory_twice ? 2U : 1U),
          "Independent old operands,forwarding,raw-bus hold,flags and pointer updates agree");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto location : {pointer - 1, pointer, pointer + 1}) {
      const bool selected = (space == DspSpace::y) == sample.input.y;
      auto value = canary;
      if (selected && location == pointer) { value = first_word; }
      if (selected && location == pointer + 1 && sample.kind == Kind::memory_twice) {
        value = second_word;
      }
      require(machine.bus().read({.space = space, .address = location}) == value,
              "Selected data and neighboring/opposite-space sentinels remain unchanged");
    }
  }
}

void dependency(const Case sample) {
  DspMachine machine;
  const auto before = prepare_case(machine, sample);
  auto run = clock(machine);
  unsigned reads = 0;
  finish(run, machine, sample, before, reads);
}

void restored(const Case sample) {
  if (sample.gap != 0) { return; }
  for (const unsigned cut : {4U, 5U, 6U, 7U}) {
    DspMachine machine;
    const auto before = prepare_case(machine, sample);
    auto run = clock(machine);
    unsigned reads = 0;
    while (run.completed_cycles() < cut) { step(run, machine, sample, reads); }
    const auto prefix_reads = reads;
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      reads = prefix_reads;
      finish(run, machine, sample, before, reads);
    }
  }
}

void unknown(const Case sample) {
  DspMachine machine;
  auto after = prepare_case(machine, sample, true);
  auto run = clock(machine);
  unsigned reads = 0;
  const auto error = checkpoint::rejects([&] { finish(run, machine, sample, after, reads); },
                                         "Unknown source must fail visibly");
  if (incoming(sample)) {
    arithmetic(after.cpu, sample.kind == Kind::incoming_source ? sample.input.b : !sample.input.b);
    after.cpu.pc += 1 + sample.gap;
    after.native_instructions += 1 + sample.gap;
    after.native_cycles += 1 + sample.gap;
  }
  // Younger AG2 may already increment R before the older E1 read fails.
  after.cpu.r.at(sample.input.reg) =
      pointer + (sample.kind == Kind::memory_twice && sample.gap == 0 ? 2U : 1U);
  require(error.contains("Unknown DSP memory read at ") && error.contains(hex(pointer, 6)) &&
              error.contains("pc=") && error.contains("omr=") && run.faulted() && reads == 0 &&
              machine.cpu().debug_state() == after,
          "Unknown data preserves only actual accepted older E2 and younger AG2 state");
}

void matrix(const auto &action) {
  constexpr std::array kinds{Kind::source_bus,
                             Kind::sum_bus,
                             Kind::alu_same,
                             Kind::alu_reverse,
                             Kind::memory_twice,
                             Kind::incoming_source,
                             Kind::incoming_sum};
  for (const unsigned pair : {1U, 4U, 6U}) {
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        const Input input{.value = operands.at(pair),
                          .status = masked_status | all_ccr | (pair == 4 ? saturation_mode : 0U),
                          .b = b,
                          .y = y};
        for (const auto kind : kinds) {
          for (const unsigned gap : {0U, 1U, 2U}) {
            action(Case{.input = input, .kind = kind, .gap = gap});
          }
        }
      }
    }
  }
}

void unsupported() {
  constexpr unsigned alu_byte = 0xff;
  constexpr unsigned fields = 7U << 16U;
  constexpr unsigned store_bit = 0x8000;
  constexpr unsigned add_a_b_byte = 0x18;
  constexpr unsigned add_b_a_byte = 0x10;
  constexpr unsigned sub_a_b_byte = 0x1c;
  constexpr unsigned sub_b_a_byte = 0x14;
  for (const bool b : {false, true}) {
    const Input input{.value = operands.at(1), .b = b};
    const auto word = opcode(input);
    require(!fetch_repeat_body(word), "ADD/load does not expand qualified REP body forms");
    const auto collision = (word & ~alu_byte) | (b ? add_a_b_byte : add_b_a_byte);
    const auto other_alu = (word & ~alu_byte) | (b ? sub_b_a_byte : sub_a_b_byte);
    const auto partial = (word & ~fields) | ((b ? 5U : 4U) << 16U);
    for (const auto rejected : {collision, other_alu, partial, word & ~store_bit}) {
      DspMachine machine;
      const std::array code{rejected};
      const auto before = prepare(machine, input, code);
      auto run = clock(machine);
      const auto error = checkpoint::rejects(
          [&] {
            while (run.completed_cycles() < bound) {
              static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
            }
          },
          "Unsupported form must fail visibly");
      require(error.contains("Unqualified automatic DSP decode") && run.faulted() &&
                  run.completed_cycles() == 2 && machine.cpu().debug_state() == before,
              "Collision/other ALU/partial/store formats fail before acquiring AG1 or CPU effects");
    }
  }
}
}

void dependencies() {
  matrix(dependency);
}

void checkpoints() {
  matrix(restored);
}

void guards() {
  matrix(unknown);
  unsupported();
}
}
