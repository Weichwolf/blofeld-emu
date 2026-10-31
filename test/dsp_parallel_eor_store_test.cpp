// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_parallel_eor_store.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
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
constexpr unsigned entry = 0x100, body = entry + 1, operand = 0x407, poison = 0x409;
constexpr unsigned word_bits = 24, word_mask = 0xffffff, byte_mask = 0xff;
constexpr unsigned ordinary_omr = 0x300, jump = 0x0af080, canary = 0x654321, illegal_word = 7;
constexpr unsigned sign_word = 0x800000, negative_flag = 8, zero_flag = 4, nzv_mask = 14;
constexpr unsigned younger_base = 0x500, last_clock = 9;
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
constexpr std::array scales{0U, 0x400U, 0x800U};
constexpr std::array flag_patterns{0U, 0xffU, 0x55U, 0xaaU};

struct Case {
  unsigned source = 0, reg = 0, variant = 0, flags = 0, scale = 0;
  bool b = false, y = false, younger = false;
};

unsigned raw_source(const Case sample) {
  return sample.younger ? younger_base + (2 * sample.reg)
                        : r_values.at((sample.variant + sample.reg) % r_values.size());
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

unsigned younger_opcode(const Case sample) {
  constexpr unsigned x0_postincrement = 0x445800;
  constexpr unsigned y_bit = 0x80000;
  return x0_postincrement | (sample.reg << 8U) | (sample.y ? 0U : y_bit);
}

void encoding(const Case sample) {
  const auto text = "eor " + std::string(source_names.at(sample.source)) + ',' +
                    (sample.b ? 'b' : 'a') + " r" + std::to_string(sample.reg) +
                    (sample.y ? ",y:>" : ",x:>") + test::number(operand);
  require(test::checkpoint::assemble(text.c_str()) ==
              std::vector<std::uint32_t>{opcode(sample), operand},
          "Independent EOR JJ/d and raw-R/absolute24 fields match assembly");
  if (sample.younger) {
    const auto text2 =
        "move x0," + std::string(sample.y ? "x:" : "y:") + "(r" + std::to_string(sample.reg) + ")+";
    require(test::checkpoint::assemble(text2.c_str()) ==
                std::vector<std::uint32_t>{younger_opcode(sample)},
            "Independent younger pointer update encoding");
  }
}

DspFetchRunInputs contract() {
  auto result = test::fetch_clock::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  result.execution = DspReferenceExecution::staged_register_boundaries;
  return result;
}

std::array<unsigned, 7> cells(const Case sample) {
  const auto other = sample.younger ? raw_source(sample) : younger_base;
  return {operand - 1, operand, operand + 1, poison, other - 1, other, other + 1};
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
  for (unsigned pc = entry; pc < entry + 16; ++pc) { machine.bus().write({.address = pc}); }
  machine.bus().write({.address = body, .value = opcode(sample)});
  machine.bus().write({.address = body + 1, .value = operand});
  if (sample.younger) {
    machine.bus().write({.address = body + 2, .value = younger_opcode(sample)});
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells(sample)) {
      machine.bus().write({.space = space, .address = cell, .value = canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == ordinary_omr &&
              before.cpu.a == a_values.at(sample.variant) &&
              before.cpu.b == b_values.at(sample.variant) &&
              before.cpu.r.at(sample.reg) == raw_source(sample) &&
              before.cpu.sr == (test::masked_status | sample.flags | sample.scale),
          "Actual seed instructions establish independent logical/raw source inputs");
  static_cast<void>(machine.begin_reference_instruction(contract().execution));
  return before;
}

unsigned alu_source(const DspState &before, const Case sample) {
  switch (sample.source) {
    case 0: return static_cast<unsigned>(before.x & word_mask);
    case 1: return static_cast<unsigned>(before.y & word_mask);
    case 2: return static_cast<unsigned>((before.x >> word_bits) & word_mask);
    case 3: return static_cast<unsigned>((before.y >> word_bits) & word_mask);
    default: throw Fault("Undefined own EOR source selector");
  }
}

unsigned eor_result(const DspState &before, const Case sample) {
  const auto value = sample.b ? before.b : before.a;
  return static_cast<unsigned>((value >> word_bits) & word_mask) ^ alu_source(before, sample);
}

DspDebugState expected(const DspDebugState &before, const Case sample, const unsigned clock) {
  auto result = before;
  if (clock >= 7) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  if (sample.younger && clock >= 8) { result.cpu.r.at(sample.reg) = raw_source(sample) + 1; }
  if (clock >= 8) {
    auto &value = sample.b ? result.cpu.b : result.cpu.a;
    constexpr std::uint64_t middle_mask = std::uint64_t{word_mask} << word_bits;
    const auto middle = eor_result(before.cpu, sample);
    value = (value & ~middle_mask) | (std::uint64_t{middle} << word_bits);
    result.cpu.sr = (before.cpu.sr & ~nzv_mask) | ((middle & sign_word) != 0 ? negative_flag : 0U) |
                    (middle == 0 ? zero_flag : 0U);
    result.cpu.pc += 2;
    ++result.native_instructions;
    result.native_cycles += 2;
  }
  if (sample.younger && clock >= 10) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  return result;
}

void memory(DspMachine &machine,
            const DspDebugState &before,
            const Case sample,
            const unsigned clock) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells(sample)) {
      unsigned value = canary;
      if (clock >= 7 && cell == operand && (space == DspSpace::y) == sample.y) {
        value = before.cpu.r.at(sample.reg);
      }
      if (sample.younger && clock >= 9 && cell == raw_source(sample) &&
          (space == DspSpace::y) != sample.y) {
        value = data_seed(sample, 0);
      }
      require(
          machine.bus().read({.space = space, .address = cell}) == value,
          "Exact E1 cells and every neighbor/opposite space follow independent data expectations");
    }
  }
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Own EOR absolute stream lost its pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const DspProgramWord word{.address = body, .value = opcode(sample)};
  const DspProgramWord extension{.address = body + 1, .value = operand};
  if (clock == 4) {
    require(phase.decode_accepted && phase.decode == word && phase.decode_words == 2,
            "Genuine captured two-word Decode");
    machine.bus().write({.address = body, .value = illegal_word});
    machine.bus().write({.address = body + 1, .value = poison});
  }
  if (clock == 5) {
    require(phase.ag1 == word && phase.ag1_extension == extension,
            "AG1 retains the captured old absolute address");
  }
  const bool first = clock == 7;
  const bool second = sample.younger && clock == 9;
  require(phase.memory.has_value() == (first || second) && !phase.memory_source_stall &&
              phase.agu_dependencies == 0 &&
              cycle.bus.core_access_count == (clock == 1 ? 0U : 1U) + (first || second ? 1U : 0U),
          "Only genuine F2 and one native write per selected E1");
  if (first) {
    if (!phase.memory) { throw Fault("EOR absolute E1 lost its real store"); }
    const auto &effect = *phase.memory;
    require(effect.address.kind == DspAddressStageKind::absolute_long &&
                !effect.address.pointer_update && effect.data.kind == DspNativeAccessKind::write &&
                effect.data.target.address == operand &&
                effect.data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                effect.data.value == before.cpu.r.at(sample.reg),
            "E1 stores raw R without flags,limiting,address substitution or pointer update");
  }
  const auto actual = machine.cpu().debug_state();
  const auto wanted = expected(before, sample, clock);
  if (actual != wanted) {
    std::cerr << "EOR clock=" << clock << " source=" << sample.source << " r=" << sample.reg
              << " variant=" << sample.variant << " b=" << sample.b << " y=" << sample.y
              << " younger=" << sample.younger << " pc=" << hex(actual.cpu.pc, 6) << '/'
              << hex(wanted.cpu.pc, 6) << " sr=" << hex(actual.cpu.sr, 6) << '/'
              << hex(wanted.cpu.sr, 6) << " a=" << hex(actual.cpu.a, 14) << '/'
              << hex(wanted.cpu.a, 14) << " b=" << hex(actual.cpu.b, 14) << '/'
              << hex(wanted.cpu.b, 14) << '\n';
  }
  require(actual == wanted,
          "Whole CPU/CCR/stack/native counters follow primary EOR and raw-R expectations");
  require(run.elapsed_ticks() == clock * period, "Every elapsed period stays exact");
  memory(machine, before, sample, clock);
}

unsigned windows = 0, restores = 0, clones = 0;

unsigned end(const Case sample) {
  return sample.younger ? 10U : last_clock;
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
      sample.scale == 0 && ((sample.source == 2 && sample.reg == 2 && sample.variant == 0 &&
                             sample.flags == 0xaa && !sample.b && !sample.y) ||
                            (sample.source == 1 && sample.reg == 7 && sample.variant == 1 &&
                             sample.flags == 0x55 && sample.b && sample.y));
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
  for (unsigned variant = 0; variant < a_values.size(); ++variant) {
    for (const auto scale : scales) {
      for (const auto flags : flag_patterns) {
        sample.variant = variant;
        sample.scale = scale;
        sample.flags = flags;
        one(sample);
      }
    }
  }
}

void matrix() {
  for (unsigned source = 0; source < 4; ++source) {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const bool b : {false, true}) {
        for (const bool y : {false, true}) {
          variants({.source = source, .reg = reg, .b = b, .y = y});
        }
      }
    }
  }
}

void exhaustive_flags() {
  for (unsigned source = 0; source < 4; ++source) {
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        for (unsigned flags = 0; flags <= byte_mask; ++flags) {
          one({.source = source, .reg = 6, .variant = 7, .flags = flags, .b = b, .y = y});
        }
      }
    }
  }
}

void younger_variants(Case sample) {
  for (unsigned variant = 0; variant < a_values.size(); ++variant) {
    for (const auto flags : {0x55U, 0xaaU}) {
      sample.variant = variant;
      sample.flags = flags;
      one(sample);
    }
  }
}

void younger() {
  for (unsigned source = 0; source < 4; ++source) {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const bool b : {false, true}) {
        for (const bool y : {false, true}) {
          younger_variants({.source = source, .reg = reg, .b = b, .y = y, .younger = true});
        }
      }
    }
  }
}

void guards() {
  for (unsigned source = 0; source < 4; ++source) {
    const auto word = opcode({.source = source});
    require(parallel_eor_address_store(word) && !fetch_repeat_body(word),
            "EOR absolute store never broadens REP");
    constexpr unsigned w_bit = 0x8000;
    require(!decode_memory_stage(word | w_bit), "EOR absolute reads remain a separate family");
  }
  std::cout << "EOR_ABSOLUTE guards=8 passed\n";
}
}

int main() {
  try {
    std::cout << "EOR_ABSOLUTE begin\n" << std::flush;
    matrix();
    exhaustive_flags();
    std::cout << "EOR_ABSOLUTE base=" << windows << " passed\n" << std::flush;
    younger();
    guards();
    std::cout << "EOR_ABSOLUTE windows=" << windows << " restores=" << restores
              << " clones=" << clones << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "EOR_ABSOLUTE windows=" << windows << ": " << error.what() << '\n';
    return 1;
  }
}
