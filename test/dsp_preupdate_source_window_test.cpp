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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned body = boot + 20;
constexpr unsigned data = 0x400, canary = 0x654321, ordinary_omr = 0x300;
constexpr unsigned producer_pc = body + 1;
constexpr unsigned producer_e1 = 7;
constexpr unsigned seed_cycles = 20;
constexpr unsigned producer_e2 = 8;
constexpr std::uint64_t initial = 0x123456abcdef;
constexpr std::array seeds{0x50f400U, 0x54f400U, 0x52f400U, 0x51f400U, 0x55f400U, 0x53f400U};
constexpr std::array clear_opcodes{0x200013U, 0x20001bU};
constexpr std::array full_opcodes{0x2e0100U, 0x2f0100U};
constexpr std::array part_opcodes{0x2c0100U, 0x2d0100U};
// FM13-118/122,Table12-14: full or partial A/B scalar stores.
constexpr std::array a_store_bases{0x504000U, 0x544000U, 0x524000U, 0x564000U};
constexpr unsigned b_bit = 0x10000, mode_shift = 11;
constexpr unsigned y_bit = 0x80000;
constexpr std::uint64_t fractional_one = 0x010000000000;
constexpr std::uint64_t integer_one_high = 0x000001abcdef;
constexpr unsigned extension_mask = 0xff;
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned clear_flags = 0x14;
enum class Producer : std::uint8_t { arithmetic, full, part };

struct Input {
  bool source_b = false;
  bool y = false;
  bool same = true;
  unsigned gap = 0;
  unsigned mode = 5;
  unsigned part = 3;
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
  // Source readiness and the preupdate wait constrain the same earliest E1.
  return 8 + input.gap + std::max(1U, stall(input));
}

unsigned last(const Input input) {
  return 12 + input.gap + std::max(1U, stall(input));
}

unsigned producer_opcode(const Input input) {
  const bool b = writes_b(input);
  switch (input.producer) {
    case Producer::arithmetic: return clear_opcodes.at(static_cast<std::size_t>(b));
    case Producer::full: return full_opcodes.at(static_cast<std::size_t>(b));
    case Producer::part: return part_opcodes.at(static_cast<std::size_t>(b));
    default: throw Fault("Undefined own producer");
  }
}

std::uint64_t producer_value(const Input input) {
  switch (input.producer) {
    case Producer::arithmetic: return 0;
    case Producer::full: return fractional_one;
    case Producer::part: return integer_one_high;
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
  program.immediate("r0", input.mode == 5 ? data - 3 : data + 1);
  program.immediate("n0", 3);
  program.word(sr_move);
  program.word(masked_status);
  program.word(0);
  program.word(producer_opcode(input));
  for (unsigned gap = 0; gap < input.gap; ++gap) { program.word(0); }
  program.word(a_store_bases.at(input.part) | (input.source_b ? b_bit : 0U) |
               (input.mode << mode_shift) | (input.y ? y_bit : 0U));
  for (unsigned index = 0; index < 32; ++index) { program.word(0); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 10);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == initial && before.cpu.b == initial &&
              before.cpu.sr == masked_status && before.native_instructions == 10 &&
              before.native_cycles == seed_cycles &&
              before.cpu.r[0] == (input.mode == 5 ? data - 3 : data + 1) &&
              before.cpu.m[0] == word_mask && before.cpu.omr == ordinary_omr,
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

unsigned store_value(const std::uint64_t source, const unsigned part) {
  if (part == 3) { return static_cast<unsigned>(source >> word_bits); }
  auto value = static_cast<unsigned>((source >> (part * word_bits)) &
                                     (part == 2 ? extension_mask : word_mask));
  constexpr unsigned extension_sign = 0x80;
  constexpr unsigned extension_fill = 0xffff00;
  if (part == 2 && (value & extension_sign) != 0) { value |= extension_fill; }
  return value;
}

struct Oracle {
  DspDebugState expected;
  unsigned stalls = 0;
  unsigned retired = 0;
  unsigned stores = 0;
};

void cells(DspMachine &machine, const Input input, const std::uint64_t period) {
  const auto source = input.same ? producer_value(input) : initial;
  const auto stored = store_value(source, input.part);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned address : {data - 1, data, data + 1}) {
      const bool selected =
          (space == DspSpace::y) == input.y && address == data && period >= consumer_e1(input);
      const auto actual = machine.bus().read({.space = space, .address = address});
      const auto wanted = selected ? stored : canary;
      if (actual != wanted) {
        std::cerr << "source-part=" << input.part << " mode=" << input.mode
                  << " same=" << input.same << " gap=" << input.gap << " period=" << period
                  << " memory=" << hex(actual, 6) << " expected=" << hex(wanted, 6) << '\n';
      }
      require(actual == wanted,
              "Actual memory matches the literal accumulator source and preserves neighbors");
    }
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
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
  if (cycle.completed_cycles == producer_e1 + input.gap) {
    oracle.expected.cpu.r[0] = input.mode == 5 ? data - 3 : data;
  }
  if (phase.memory) {
    ++oracle.stores;
    if (cycle.completed_cycles != consumer_e1(input)) {
      std::cerr << "source-part=" << input.part << " mode=" << input.mode << " same=" << input.same
                << " gap=" << input.gap << " actual-data-period=" << cycle.completed_cycles
                << " expected=" << consumer_e1(input) << " interlocks=" << oracle.stalls << '\n';
    }
    require(cycle.completed_cycles == consumer_e1(input) && phase.execute1 &&
                phase.execute1->address == consumer_pc(input),
            "The once-only native scalar store observes the real post-interlock source");
  }
  cells(machine, input, cycle.completed_cycles);
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "The literal producer/consumer/NOP path retains exact native retirement order");
    if (oracle.expected.cpu.pc == producer_pc && arithmetic) {
      oracle.expected.cpu.sr = masked_status | clear_flags;
    }
    oracle.expected.native_cycles += oracle.expected.cpu.pc == consumer_pc(input) ? 2U : 1U;
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
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
  for (unsigned cut = 6; cut < last(input); ++cut) {
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

void windows(Input input) {
  for (const bool same : {false, true}) {
    for (unsigned gap = 0; gap <= 1; ++gap) {
      input.same = same;
      input.gap = gap;
      one_case(input);
    }
  }
}

}

int main() {
  try {
    for (const bool source_b : {false, true}) {
      for (const bool y : {false, true}) {
        for (const unsigned mode : {5U, 7U}) {
          for (unsigned part = 0; part < 4; ++part) {
            for (const auto producer : {Producer::arithmetic, Producer::full, Producer::part}) {
              const Input input{
                  .source_b = source_b, .y = y, .mode = mode, .part = part, .producer = producer};
              windows(input);
              restores(input);
            }
          }
        }
      }
    }
    std::cout << "384 preupdate overlapping-source windows,672 cuts/1344 suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
