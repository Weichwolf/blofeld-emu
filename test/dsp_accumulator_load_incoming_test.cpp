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
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned seed_instructions = 12, seed_cycles = 24, body = boot + seed_cycles;
constexpr unsigned producer_pc = body + 1, source = 0x407, wrapped_source = 0x400;
constexpr unsigned input_value = 0x923456, canary = 0x654321, ordinary_omr = 0x300;
constexpr unsigned modifier_input = 0x812345, modulo = 0xff0007, initial_ccr = 0xc1;
constexpr unsigned clear_ccr = 0x14, preserved_ccr = 0xc1, extension_mask = 0xff;
constexpr unsigned b_bit = 0x10000, y_bit = 0x80000, postincrement_r0 = 0x1800;
constexpr std::uint64_t initial = 0x80800000abcdef, accumulator_mask = 0xffffffffffffff;
constexpr std::uint64_t loaded_full = 0xff923456000000;
constexpr std::uint64_t low_written = 0x80800000000001;
constexpr std::uint64_t high_written = 0x80000001abcdef;
constexpr std::uint64_t extension_written = 0x01800000abcdef;
constexpr std::uint64_t modifier_written = 0xff812345000000;
// FM Table12-14: standalone accumulator W=1/EA=1 load destination fields.
constexpr std::array load_words{
    0x50c000U, 0x54c000U, 0x52c000U, 0x51c000U, 0x55c000U, 0x53c000U, 0x56c000U, 0x57c000U};
// Own literal CLR and short full/A0/A1/A2 writes; low8 immediate is integer for parts.
constexpr std::array clear_words{0x200013U, 0x20001bU};
constexpr std::array move_words{0x2e0100U, 0x280100U, 0x2c0100U, 0x2a0100U};
constexpr std::array modifier_words{0x044ea1U, 0x044fa1U};
enum class Writer : std::uint8_t { clear, full, low, high, extension, modifier };
constexpr std::array writers{
    Writer::clear, Writer::full, Writer::low, Writer::high, Writer::extension, Writer::modifier};

struct Input {
  Writer writer = Writer::clear;
  bool writer_b = false, input_y = false;
  unsigned destination = 0, gap = 0, ccr = 0;
};

bool load_b(const Input input) {
  return input.destination == 7 || (input.destination >= 3 && input.destination < 6);
}

unsigned consumer_pc(const Input input) {
  return body + 2 + input.gap;
}

unsigned last(const Input input) {
  return 12 + input.gap;
}

unsigned producer_opcode(const Input input) {
  if (input.writer == Writer::clear) { return clear_words.at(input.writer_b ? 1U : 0U); }
  if (input.writer == Writer::modifier) { return modifier_words.at(input.writer_b ? 1U : 0U); }
  return move_words.at(static_cast<unsigned>(input.writer) - 1) | (input.writer_b ? b_bit : 0U);
}

std::uint64_t producer_value(const Input input) {
  switch (input.writer) {
    case Writer::clear: return 0;
    case Writer::full: return 0x010000000000;
    case Writer::low: return low_written;
    case Writer::high: return high_written;
    case Writer::extension: return extension_written;
    case Writer::modifier: return modifier_written;
    default: throw Fault("Undefined own writer value");
  }
}

std::uint64_t destination_value(const Input input) {
  if (input.destination >= 6) { return loaded_full; }
  const auto before = input.writer_b == load_b(input) ? producer_value(input) : initial;
  const auto part = input.destination % 3;
  const auto shift = part * word_bits;
  const auto mask = static_cast<std::uint64_t>(part == 2 ? extension_mask : word_mask) << shift;
  return (before & (accumulator_mask ^ mask)) |
         ((static_cast<std::uint64_t>(input_value) << shift) & mask);
}

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram program;
  for (const bool b : {false, true}) {
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(std::string(b ? "b" : "a") + std::to_string(part),
                        (initial >> (part * word_bits)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  program.immediate("omr", ordinary_omr);
  program.immediate("m1", modifier_input);
  program.immediate("r0", source);
  program.immediate("n0", 3);
  program.immediate("m0", modulo);
  program.immediate("sr", masked_status | input.ccr);
  program.word(0);
  program.word(producer_opcode(input));
  for (unsigned gap = 0; gap < input.gap; ++gap) { program.word(0); }
  program.word(load_words.at(input.destination) | postincrement_r0 | (input.input_y ? y_bit : 0U));
  for (unsigned index = 0; index < 32; ++index) { program.word(0); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == initial && before.cpu.b == initial &&
              before.cpu.sr == (masked_status | input.ccr) && before.cpu.omr == ordinary_omr &&
              before.cpu.r[0] == source && before.cpu.m[0] == modulo && before.cpu.n[0] == 3 &&
              before.cpu.m[1] == modifier_input &&
              before.native_instructions == seed_instructions &&
              before.native_cycles == seed_cycles,
          "Actual native seed instructions establish exact accumulators,modes and literal cost");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {source - 1, source, source + 1}) {
      const bool selected = address == source && (space == DspSpace::y) == input.input_y;
      machine.bus().write(
          {.space = space, .address = address, .value = selected ? input_value : canary});
    }
  }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

struct Oracle {
  DspDebugState expected;
  unsigned reads = 0, transfers = 0, retired = 0;
};

void memory(DspMachine &machine, const Input input) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {source - 1, source, source + 1}) {
      const bool selected = address == source && (space == DspSpace::y) == input.input_y;
      require(machine.bus().read({.space = space, .address = address}) ==
                  (selected ? input_value : canary),
              "The younger native read preserves every source/neighbor/opposite-space word");
    }
  }
}

void receipt(const DspPipelineCycle &phase,
             const Input input,
             const unsigned period,
             Oracle &oracle) {
  const bool transfer = input.writer != Writer::clear;
  require(phase.transfer.has_value() == (transfer && period == 7),
          "Only an older genuine move publishes an E1 transfer; CLR stays deferred to E2");
  if (phase.transfer) {
    ++oracle.transfers;
    require(phase.execute1 && phase.execute1->address == producer_pc,
            "The real older transfer receipt retains its original producer program address");
  }
  require(phase.memory.has_value() == (period == 8 + input.gap) && !phase.memory_source_stall,
          "A new load destination inserts no accumulator-source hold or fabricated data receipt");
  if (!phase.memory) { return; }
  ++oracle.reads;
  const auto &access = phase.memory->data;
  require(!phase.memory->additional_data && access.kind == DspNativeAccessKind::read &&
              phase.execute1 && phase.execute1->address == consumer_pc(input) &&
              access.target.address == source &&
              access.target.space == (input.input_y ? DspSpace::y : DspSpace::x) &&
              access.value == input_value,
          "The once-only younger E1 reads its actual independent source and original AG1 address");
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto period = static_cast<unsigned>(cycle.completed_cycles);
  require(phase.retirement_published == (period >= 7),
          "Each one-word anchor/writer/gap/load/successor retires at its literal nominal E2");
  if (period == (input.writer == Writer::clear ? 8U : 7U)) {
    auto &target = input.writer_b ? oracle.expected.cpu.b : oracle.expected.cpu.a;
    target = producer_value(input);
    if (input.writer == Writer::clear) {
      oracle.expected.cpu.sr = masked_status | (input.ccr & preserved_ccr) | clear_ccr;
    }
  }
  if (period == 7 + input.gap) { oracle.expected.cpu.r[0] = wrapped_source; }
  if (period == 8 + input.gap) {
    auto &target = load_b(input) ? oracle.expected.cpu.b : oracle.expected.cpu.a;
    target = destination_value(input);
  }
  receipt(phase, input, period, oracle);
  memory(machine, input);
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Every real retirement preserves the own literal instruction sequence");
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    ++oracle.retired;
  }
  const auto actual = machine.cpu().debug_state();
  if (actual != oracle.expected) {
    std::cerr << "period=" << period << " writer=" << static_cast<unsigned>(input.writer)
              << " destination=" << input.destination << " gap=" << input.gap
              << " SR=" << hex(actual.cpu.sr, 6) << '/' << hex(oracle.expected.cpu.sr, 6)
              << " A=" << hex(actual.cpu.a, 14) << '/' << hex(oracle.expected.cpu.a, 14)
              << " B=" << hex(actual.cpu.b, 14) << '/' << hex(oracle.expected.cpu.b, 14) << '\n';
  }
  require(
      actual == oracle.expected,
      "New load fields/older untouched parts,older CLR CCR,younger AG2 and counters all survive");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < last(input)) { step(run, machine, input, oracle); }
  require(oracle.reads == 1 && oracle.transfers == (input.writer == Writer::clear ? 0U : 1U) &&
              oracle.retired == 6 + input.gap &&
              run.elapsed_ticks() == last(input) * pipeline::period,
          "Every complete incoming-writer window retains exact native publication ownership/cost");
}

void one(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input)};
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, oracle);
}

void snapshots(const Input input) {
  for (unsigned cut = 6; cut <= 10; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input)};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto before = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = before;
      finish(run, machine, input, oracle);
    }
  }
}

void variants(Input sample) {
  for (const auto ccr : {0U, initial_ccr}) {
    for (unsigned gap = 0; gap <= 1; ++gap) {
      sample.ccr = ccr;
      sample.gap = gap;
      one(sample);
    }
  }
  for (unsigned gap = 0; gap <= 1; ++gap) {
    sample.ccr = initial_ccr;
    sample.gap = gap;
    snapshots(sample);
  }
}
}

int main() {
  try {
    for (const auto writer : writers) {
      for (const bool writer_b : {false, true}) {
        for (unsigned destination = 0; destination < load_words.size(); ++destination) {
          for (const bool input_y : {false, true}) {
            variants({.writer = writer,
                      .writer_b = writer_b,
                      .input_y = input_y,
                      .destination = destination});
          }
        }
      }
    }
    std::cout
        << "768 older CLR/full/A0/A1/A2/M1 writer-to-load windows,1920 cuts/3840 suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
