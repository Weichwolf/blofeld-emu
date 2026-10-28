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
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned body = boot + 22;
constexpr unsigned data = 0x400, canary = 0x654321, ordinary_omr = 0x300;
constexpr unsigned producer_pc = body + 1;
constexpr unsigned producer_e1 = 7;
constexpr unsigned seed_cycles = 22;
constexpr unsigned producer_e2 = 8;
constexpr std::uint64_t initial = 0x80800000abcdef;
constexpr std::array seeds{0x50f400U, 0x54f400U, 0x52f400U, 0x51f400U, 0x55f400U, 0x53f400U};
constexpr std::array clear_opcodes{0x200013U, 0x20001bU};
constexpr std::array full_opcodes{0x2e0100U, 0x2f0100U};
constexpr std::array part_opcodes{0x2c0100U, 0x2d0100U};
// FM13-118/122,Table12-14: A0/A1/A2 to (R0)+; B is bit16.
constexpr std::array read_opcodes{0x505800U, 0x545800U, 0x525800U};
constexpr unsigned b_bit = 0x10000;
constexpr std::uint64_t modifier_value = 0xffffffff000000;
constexpr std::array modifier_opcodes{0x044ea1U, 0x044fa1U};
constexpr unsigned y_bit = 0x80000;
constexpr std::uint64_t fractional_one = 0x010000000000;
constexpr std::uint64_t integer_one_high = 0x80000001abcdef;
constexpr unsigned extension_mask = 0xff;
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned clear_flags = 0x14;
enum class Producer : std::uint8_t { arithmetic, full, part, modifier };

struct Input {
  bool source_b = false;
  bool y = false;
  bool same = true;
  unsigned gap = 0;
  unsigned source_part = 0;
  Producer producer = Producer::arithmetic;
};

bool writes_b(const Input input) {
  return input.same ? input.source_b : !input.source_b;
}

unsigned stall(const Input input) {
  return input.same && input.gap == 0 ? 1U : 0U;
}

unsigned consumer_pc(const Input input) {
  return body + 2 + input.gap;
}

unsigned consumer_e1(const Input input) {
  return 8 + input.gap + stall(input);
}

unsigned last(const Input input) {
  return 12 + input.gap + stall(input);
}

unsigned producer_opcode(const Input input) {
  const bool b = writes_b(input);
  switch (input.producer) {
    case Producer::arithmetic: return clear_opcodes.at(static_cast<std::size_t>(b));
    case Producer::full: return full_opcodes.at(static_cast<std::size_t>(b));
    case Producer::part: return part_opcodes.at(static_cast<std::size_t>(b));
    case Producer::modifier: return modifier_opcodes.at(static_cast<std::size_t>(b));
    default: throw Fault("Undefined own producer");
  }
}

std::uint64_t producer_value(const Input input) {
  switch (input.producer) {
    case Producer::arithmetic: return 0;
    case Producer::full: return fractional_one;
    case Producer::part: return integer_one_high;
    case Producer::modifier: return modifier_value;
    default: throw Fault("Undefined own producer value");
  }
}

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram program;
  for (unsigned index = 0; index < seeds.size(); ++index) {
    program.word(seeds[index]);
    const auto part = index % 3;
    program.word(static_cast<unsigned>((initial >> (part * word_bits)) &
                                       (part == 2 ? extension_mask : word_mask)));
  }
  program.immediate("omr", ordinary_omr);
  program.immediate("m1", word_mask);
  program.immediate("r0", data);
  program.immediate("n0", 3);
  program.word(sr_move);
  program.word(masked_status);
  program.word(0);
  program.word(producer_opcode(input));
  for (unsigned gap = 0; gap < input.gap; ++gap) { program.word(0); }
  program.word(read_opcodes.at(input.source_part) | (input.source_b ? b_bit : 0U) |
               (input.y ? y_bit : 0U));
  for (unsigned index = 0; index < 32; ++index) { program.word(0); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 11);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == initial && before.cpu.b == initial &&
              before.cpu.sr == masked_status && before.native_instructions == 11 &&
              before.native_cycles == seed_cycles && before.cpu.r[0] == data &&
              before.cpu.omr == ordinary_omr,
          "Native seeds establish both full sources, address state and independent exact CPU cost");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

struct Oracle {
  DspDebugState expected;
  unsigned stalls = 0;
  unsigned retired = 0;
  unsigned stores = 0;
};

unsigned retired_count(const Input input, const unsigned period) {
  unsigned count = 0;
  for (unsigned index = 0; index < 6 + input.gap; ++index) {
    const auto ready = 7 + index + (index >= 2 + input.gap ? stall(input) : 0U);
    if (period >= ready) { ++count; }
  }
  return count;
}

unsigned stored_value(const Input input) {
  const auto source = input.same ? producer_value(input) : initial;
  constexpr unsigned extension_sign = 0x80;
  constexpr unsigned extension_fill = 0xffff00;
  const auto shift = input.source_part * word_bits;
  auto stored = static_cast<unsigned>((source >> shift) &
                                      (input.source_part == 2 ? extension_mask : word_mask));
  if (input.source_part == 2 && (stored & extension_sign) != 0) { stored |= extension_fill; }
  return stored;
}

void check_cells(DspMachine &machine, const Input input, const unsigned period) {
  const auto stored = stored_value(input);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      const bool selected =
          (space == DspSpace::y) == input.y && address == data && period >= consumer_e1(input);
      require(
          machine.bus().read({.space = space, .address = address}) == (selected ? stored : canary),
          "Actual memory matches the independent raw/sign-extended part and preserves neighbors");
    }
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  require(
      phase.retirement_published ==
          (retired_count(input, period) != retired_count(input, period - 1)),
      "Independent one-word instruction costs and source holds determine every retirement period");
  if (cycle.completed_cycles == producer_e2) {
    require(phase.memory_source_stall == (stall(input) != 0),
            "Independent FM/vendor expectation requires exactly one same-accumulator interlock");
  }
  if (phase.memory_source_stall) {
    ++oracle.stalls;
    require(cycle.completed_cycles == producer_e2 && phase.execute1 &&
                phase.execute1->address == consumer_pc(input) && !phase.memory &&
                !phase.execution_captured,
            "The real consumer E1 stays held while the older native E2 result publishes");
  }
  const bool arithmetic = input.producer == Producer::arithmetic;
  if (cycle.completed_cycles == (arithmetic ? producer_e2 : producer_e1)) {
    auto &target = writes_b(input) ? oracle.expected.cpu.b : oracle.expected.cpu.a;
    target = producer_value(input);
  }
  if (cycle.completed_cycles == producer_e1 + input.gap) { oracle.expected.cpu.r[0] = data + 1; }
  if (phase.memory) {
    ++oracle.stores;
    require(cycle.completed_cycles == consumer_e1(input) && phase.execute1 &&
                phase.execute1->address == consumer_pc(input),
            "The once-only native scalar store observes the real post-interlock source");
  }
  check_cells(machine, input, period);
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "The literal producer/consumer/NOP path retains exact native retirement order");
    if (oracle.expected.cpu.pc == producer_pc && arithmetic) {
      oracle.expected.cpu.sr = masked_status | clear_flags;
    }
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period matches full literal CPU/stack/flags/destination/counter expectations");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < last(input)) { step(run, machine, input, oracle); }
  require(oracle.retired == 6 + input.gap && oracle.stalls == stall(input) && oracle.stores == 1 &&
              run.elapsed_ticks() == last(input) * pipeline::period,
          "The complete unchanged-model window contains only its independently required stall");
}

void one_case(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input)};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void restores(const Input input) {
  for (unsigned cut = 6; cut <= 11; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input)};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto checkpoint = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(checkpoint, machine);
      oracle = prefix;
      finish(run, machine, input, oracle);
    }
  }
}

void windows(const bool source_b,
             const bool y,
             const Producer producer,
             const unsigned source_part) {
  for (const bool same : {false, true}) {
    for (unsigned gap = 0; gap <= 1; ++gap) {
      one_case({.source_b = source_b,
                .y = y,
                .same = same,
                .gap = gap,
                .source_part = source_part,
                .producer = producer});
    }
  }
}

}

int main() {
  try {
    for (const bool source_b : {false, true}) {
      for (const bool y : {false, true}) {
        for (const auto producer :
             {Producer::arithmetic, Producer::full, Producer::part, Producer::modifier}) {
          for (unsigned part = 0; part < 3; ++part) {
            windows(source_b, y, producer, part);
            restores({.source_b = source_b, .y = y, .source_part = part, .producer = producer});
          }
        }
      }
    }
    std::cout << "192 independent partial-store windows,288 checkpoint cuts/576 suffixes passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
