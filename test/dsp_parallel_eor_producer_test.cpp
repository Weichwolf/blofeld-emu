// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned entry = 0x100, operand = 0x407, poison = 0x409, tail = 0x40b;
constexpr unsigned word_bits = 24, word_mask = 0xffffff, byte_mask = 0xff;
constexpr unsigned ordinary_omr = 0x300, jump = 0x0af080, canary = 0x654321, illegal_word = 7;
constexpr unsigned sign_word = 0x800000, negative_flag = 8, zero_flag = 4, nzv_mask = 14;
constexpr unsigned last_clock = 13;
constexpr std::uint64_t period = 45;
constexpr std::array<std::uint64_t, 8> a_values{0x12000000123456,
                                                0x7f7fffffabcd12,
                                                0x00800000765432,
                                                0xffffffffffffff,
                                                0xff800000000000,
                                                0x00123456000000,
                                                0x807fffff000001,
                                                0x7fabcdef123456};
constexpr std::array<std::uint64_t, 8> b_values{0x81000000654321,
                                                0x80800001000001,
                                                0xff7fffff123456,
                                                0x00000000000001,
                                                0x7f000000ffffff,
                                                0x8055aa55999999,
                                                0x7f800000f00000,
                                                0xffabcdef765432};
constexpr std::array data_values{
    0U, word_mask, sign_word, 0x7fffffU, 0x123456U, 0xabcdefU, 0x55aa55U, 0xaaaaaaU};
constexpr std::array r_values{0U, 1U, 0x7fffffU, sign_word, 0xabcdefU, word_mask};
constexpr std::array source_names{"x0", "y0", "x1", "y1"};

struct Case {
  unsigned source = 0, reg = 0, variant = 0, flags = 0, scale = 0;
  unsigned gap = 0;
  bool b = false, y = false, reverse = false;
};

constexpr std::array heads{
    0U, 1U, 0x7fffffU, sign_word, 0xabcdefU, word_mask, 0x123456U, 0xaaaaaaU};
constexpr std::array tails{0U, 1U, word_mask, 0U, 0x123456U, 0U, 0x123456U, 0x555555U};

unsigned body(const Case sample) {
  return entry + 5 + sample.gap;
}

unsigned raw_source(const Case sample) {
  return r_values.at((sample.variant + sample.source) % r_values.size());
}

unsigned producer_word(const Case sample, const bool data) {
  constexpr std::array fields{4U, 6U, 5U, 7U};
  const unsigned accumulator_field = sample.b ? 13U : 12U;
  const unsigned field = data ? fields.at(sample.source) : accumulator_field;
  // FM Table12-14 and MOVE13-118/122: 01ddSddd W=1 EA=1 MMM=110 RRR=000.
  constexpr unsigned absolute_read = 0x40f000;
  constexpr unsigned high_field_mask = 0x18;
  constexpr unsigned high_field_shift = 17;
  constexpr unsigned low_field_mask = 7;
  constexpr unsigned low_field_shift = 16;
  constexpr unsigned y_bit = 0x80000;
  return absolute_read | ((field & high_field_mask) << high_field_shift) |
         ((field & low_field_mask) << low_field_shift) | (sample.y ? y_bit : 0U);
}

unsigned data_seed(const Case sample, const unsigned reg) {
  return data_values.at((sample.variant + 2 * reg) % data_values.size());
}

unsigned opcode(const Case sample) {
  constexpr unsigned base = 0x607043;
  constexpr unsigned y_bit = 0x80000;
  return base | (sample.reg << 16U) | (sample.source << 4U) | (sample.b ? 8U : 0U) |
         (sample.y ? y_bit : 0U);
}

void encoding(const Case sample) {
  const auto *const destination = sample.b ? "b" : "a";
  const auto *const space = sample.y ? "y:>" : "x:>";
  const auto text = "eor " + std::string(source_names.at(sample.source)) + ',' + destination +
                    " r" + std::to_string(sample.reg) + ',' + space + test::number(operand);
  require(test::checkpoint::assemble(text.c_str()) ==
              std::vector<std::uint32_t>{opcode(sample), operand},
          "Independent EOR/source/absolute24 fields match assembly");
  for (const bool data : {false, true}) {
    const auto reg =
        data ? std::string(source_names.at(sample.source)) : std::string(destination) + '1';
    const unsigned address = data ? tail : operand;
    const auto load = "move " + std::string(space) + test::number(address) + ',' + reg;
    require(test::checkpoint::assemble(load.c_str()) ==
                std::vector<std::uint32_t>{producer_word(sample, data), address},
            "Independent absolute-load producer fields match assembly");
  }
}

DspFetchRunInputs contract() {
  auto result = test::fetch_clock::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  result.execution = DspReferenceExecution::staged_register_boundaries;
  return result;
}

std::array<unsigned, 7> cells() {
  return {operand - 1, operand, operand + 1, poison, tail - 1, tail, tail + 1};
}

void seed(test::DspProgram &code, const Case sample) {
  code.immediate("omr", ordinary_omr);
  code.immediate("sz", 0);
  for (unsigned i = 0; i < 8; ++i) {
    code.immediate("r" + std::to_string(i), i == sample.reg ? raw_source(sample) : canary + i);
    code.immediate("n" + std::to_string(i), i + 1);
    code.immediate("m" + std::to_string(i), word_mask);
  }
  constexpr std::array names{"x0", "x1", "y0", "y1"};
  for (unsigned i = 0; i < names.size(); ++i) { code.immediate(names.at(i), data_seed(sample, i)); }
  for (const bool b : {false, true}) {
    const auto value = b ? b_values.at(sample.variant) : a_values.at(sample.variant);
    for (unsigned i = 0; i < 3; ++i) {
      code.immediate(std::string(b ? "b" : "a") + std::to_string(i),
                     (value >> (i * word_bits)) & (i == 2 ? byte_mask : word_mask));
    }
  }
  code.immediate("sr", test::masked_status | sample.flags | sample.scale);
  for (unsigned i = 0; i < 5; ++i) { code.word(0); }
  code.word(jump);
  code.word(entry);
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  test::DspProgram code;
  seed(code, sample);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned pc = entry; pc < entry + 32; ++pc) { machine.bus().write({.address = pc}); }
  for (unsigned i = 0; i < 2; ++i) {
    const bool data = (i == 0) == sample.reverse;
    const unsigned pc = entry + 1 + (2 * i);
    machine.bus().write({.address = pc, .value = producer_word(sample, data)});
    machine.bus().write({.address = pc + 1, .value = data ? tail : operand});
  }
  machine.bus().write({.address = body(sample), .value = opcode(sample)});
  machine.bus().write({.address = body(sample) + 1, .value = operand});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells()) {
      unsigned value = canary;
      if ((space == DspSpace::y) == sample.y) {
        if (cell == operand) { value = heads.at(sample.variant); }
        if (cell == tail) { value = tails.at(sample.variant); }
      }
      machine.bus().write({.space = space, .address = cell, .value = value});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == ordinary_omr &&
              before.cpu.a == a_values.at(sample.variant) &&
              before.cpu.b == b_values.at(sample.variant) &&
              before.cpu.r.at(sample.reg) == raw_source(sample) &&
              before.cpu.sr == (test::masked_status | sample.flags | sample.scale),
          "Actual seed instructions establish settled independent producer inputs");
  static_cast<void>(machine.begin_reference_instruction(contract().execution));
  return before;
}

void load(DspState &state, const Case sample, const bool data) {
  constexpr std::uint64_t mask = std::uint64_t{word_mask} << word_bits;
  if (!data) {
    auto &value = sample.b ? state.b : state.a;
    value = (value & ~mask) | (std::uint64_t{heads.at(sample.variant)} << word_bits);
    return;
  }
  auto &value = sample.source == 0 || sample.source == 2 ? state.x : state.y;
  const unsigned shift = sample.source >= 2 ? word_bits : 0;
  value = (value & ~(std::uint64_t{word_mask} << shift)) |
          (std::uint64_t{tails.at(sample.variant)} << shift);
}

DspDebugState expected(const DspDebugState &before, const Case sample, const unsigned clock) {
  auto result = before;
  for (unsigned i = 0; i < 2; ++i) {
    if (clock >= 7 + (2 * i)) { load(result.cpu, sample, (i == 0) == sample.reverse); }
    if (clock >= 8 + (2 * i)) {
      result.cpu.pc += 2;
      ++result.native_instructions;
      result.native_cycles += 2;
    }
  }
  if (clock >= 7) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  for (unsigned i = 0; i < sample.gap; ++i) {
    if (clock >= 12 + i) {
      ++result.cpu.pc;
      ++result.native_instructions;
      ++result.native_cycles;
    }
  }
  if (clock >= 12 + sample.gap) {
    auto &value = sample.b ? result.cpu.b : result.cpu.a;
    constexpr std::uint64_t middle_mask = std::uint64_t{word_mask} << word_bits;
    const auto middle = heads.at(sample.variant) ^ tails.at(sample.variant);
    value = (value & ~middle_mask) | (std::uint64_t{middle} << word_bits);
    result.cpu.sr = (before.cpu.sr & ~nzv_mask) | ((middle & sign_word) != 0 ? negative_flag : 0U) |
                    (middle == 0 ? zero_flag : 0U);
    result.cpu.pc += 2;
    ++result.native_instructions;
    result.native_cycles += 2;
  }
  return result;
}

void memory(DspMachine &machine, const Case sample, const unsigned clock) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells()) {
      unsigned value = canary;
      if ((space == DspSpace::y) == sample.y) {
        if (cell == operand) {
          value = clock >= 11 + sample.gap ? raw_source(sample) : heads.at(sample.variant);
        }
        if (cell == tail) { value = tails.at(sample.variant); }
      }
      require(machine.bus().read({.space = space, .address = cell}) == value,
              "Old head/tail inputs survive until genuine publication;guards and opposite space "
              "stay exact");
    }
  }
}

void captured(DspMachine &machine,
              const DspPipelineCycle &phase,
              const Case sample,
              const unsigned clock) {
  for (unsigned i = 0; i < 3; ++i) {
    const bool data = (i == 0) == sample.reverse;
    const unsigned pc = i == 2 ? body(sample) : entry + 1 + (2 * i);
    const unsigned decoded = i == 2 ? 8 + sample.gap : 4 + (2 * i);
    const DspProgramWord word{.address = pc,
                              .value = i == 2 ? opcode(sample) : producer_word(sample, data)};
    const DspProgramWord extension{.address = pc + 1, .value = i == 2 || !data ? operand : tail};
    if (clock == decoded) {
      require(phase.decode_accepted && phase.decode == word && phase.decode_words == 2,
              "Actual captured producer/EOR Decode has the independently encoded two-word form");
      machine.bus().write({.address = pc, .value = illegal_word});
      machine.bus().write({.address = pc + 1, .value = poison});
    }
    if (clock == decoded + 1) {
      require(phase.ag1 == word && phase.ag1_extension == extension,
              "AG1 retains each captured old producer/EOR address despite backing replacement");
    }
  }
}

void accesses(const DspFetchCycleResult &cycle,
              const DspPipelineCycle &phase,
              const Case sample,
              const unsigned clock) {
  const bool first = clock == 7;
  const bool second = clock == 9;
  const bool published = clock == 11 + sample.gap;
  const bool access = first || second || published;
  require(phase.memory.has_value() == access && !phase.memory_source_stall &&
              phase.agu_dependencies == 0 &&
              cycle.bus.core_access_count == (clock == 1 ? 0U : 1U) + (access ? 1U : 0U),
          "Only genuine F2, two producer E1 reads and one EOR E1 write use the bus");
  if (access) {
    if (!phase.memory) { throw Fault("Owned producer/EOR E1 memory effect is missing"); }
    const auto &effect = *phase.memory;
    const bool data = (first == sample.reverse);
    const unsigned address = published || !data ? operand : tail;
    const unsigned loaded = data ? tails.at(sample.variant) : heads.at(sample.variant);
    const unsigned value = published ? raw_source(sample) : loaded;
    require(effect.address.kind == DspAddressStageKind::absolute_long &&
                !effect.address.pointer_update &&
                effect.data.kind ==
                    (published ? DspNativeAccessKind::write : DspNativeAccessKind::read) &&
                effect.data.target.address == address &&
                effect.data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                effect.data.value == value,
            "Exact E1 operands come from independent literal head/tail/R values");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Own producer/EOR stream lost its pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  captured(machine, phase, sample, clock);
  accesses(cycle, phase, sample, clock);
  const auto actual = machine.cpu().debug_state();
  const auto wanted = expected(before, sample, clock);
  if (actual != wanted) {
    std::cerr << "EOR_PRODUCER clock=" << clock << " source=" << sample.source
              << " r=" << sample.reg << " variant=" << sample.variant << " b=" << sample.b
              << " y=" << sample.y << " reverse=" << sample.reverse << " gap=" << sample.gap
              << " pc=" << hex(actual.cpu.pc, 6) << '/' << hex(wanted.cpu.pc, 6)
              << " sr=" << hex(actual.cpu.sr, 6) << '/' << hex(wanted.cpu.sr, 6)
              << " a=" << hex(actual.cpu.a, 14) << '/' << hex(wanted.cpu.a, 14)
              << " b=" << hex(actual.cpu.b, 14) << '/' << hex(wanted.cpu.b, 14) << '\n';
  }
  require(
      actual == wanted,
      "Whole state,old-head XOR flags and producer/EOR counters match independent expectations");
  require(run.elapsed_ticks() == clock * period, "Every producer/EOR elapsed period stays exact");
  memory(machine, sample, clock);
}

unsigned windows = 0, restores = 0, clones = 0;

unsigned end(const Case sample) {
  return last_clock + sample.gap;
}

void finish(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  while (run.completed_cycles() < end(sample)) { step(run, machine, before, sample); }
}

void one(const Case sample) {
  encoding(sample);
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(contract(), machine);
  const bool snapshots =
      (sample.source == 2 && sample.reg == 2 && sample.variant == 2 && sample.flags == 0xaa &&
       !sample.b && !sample.y && !sample.reverse && sample.gap == 0) ||
      (sample.source == 1 && sample.reg == 7 && sample.variant == 7 && sample.flags == 0x55 &&
       sample.b && sample.y && sample.reverse && sample.gap == 2);
  std::vector<DspFetchCheckpoint> cuts;
  while (run.completed_cycles() < end(sample)) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    step(run, machine, before, sample);
  }
  for (const auto &cut : cuts) {
    run.restore(cut, machine);
    finish(run, machine, before, sample);
    ++restores;
    DspMachine fresh;
    DspFetchRun clone(cut, fresh);
    finish(clone, fresh, before, sample);
    ++clones;
  }
  ++windows;
}

void variants(Case sample) {
  for (unsigned gap = 0; gap < 3; ++gap) {
    for (unsigned variant = 0; variant < heads.size(); ++variant) {
      for (const auto flags : {0x55U, 0xaaU}) {
        sample.gap = gap;
        sample.variant = variant;
        sample.reg = variant;
        sample.flags = flags;
        one(sample);
      }
    }
  }
}

void matrix() {
  for (unsigned source = 0; source < 4; ++source) {
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        for (const bool reverse : {false, true}) {
          variants({.source = source, .b = b, .y = y, .reverse = reverse});
        }
      }
    }
  }
}

}

int main() {
  try {
    std::cout << "EOR_PRODUCER begin\n" << std::flush;
    matrix();
    std::cout << "EOR_PRODUCER windows=" << windows << " restores=" << restores
              << " clones=" << clones << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "EOR_PRODUCER windows=" << windows << ": " << error.what() << '\n';
    return 1;
  }
}
