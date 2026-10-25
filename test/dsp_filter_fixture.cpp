// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_filter_fixture.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp_filter_model.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace blofeld::test::filters {
namespace {
constexpr unsigned program_base = 0x126b;
constexpr unsigned ordinary_core = 0x1337;
constexpr unsigned wide_core = 0x1353;
constexpr unsigned history_base = 0x700;
constexpr unsigned input_base = 0x800;
constexpr unsigned parameters = 0xc00;
constexpr unsigned entry_slot = 0x44;
constexpr unsigned status_slot = 0x45;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned memory_switch_delay = 3;
// DSP56300FM JMP <ea>: absolute-address extension, followed by the 24-bit target.
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned instruction_bound = 512;
constexpr unsigned guard = 0x13579b;
constexpr unsigned scratch_seed = 0x2468ac;
constexpr unsigned retained_status_mask = 0xffff00;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
constexpr std::array<unsigned, 4> entries{0x1316, 0x139e, 0x13f3, 0x1459};
constexpr std::array<unsigned, 4> cascade_entries{0x136a, 0x13c4, 0x1422, 0x1484};
static_assert(parameters - 1 > table_base + table_words);
static_assert(input_base + block_length + 1 < table_base);

std::array<unsigned, 8> guards(const bool cascade) {
  return {0,
          4,
          history_base - 1,
          history_base + (cascade ? 4U : 2U),
          input_base - 1,
          input_base + block_length + 1,
          parameters - 1,
          parameters + 2};
}

void install_harness(DspBus &bus, unsigned &stop) {
  DspProgram code;
  code.immediate("x1", memory_mode);
  // The native assembler spells control-register MOVEC forms as MOVE.
  code.emit("move x1,omr");
  for (unsigned index = 0; index < memory_switch_delay; ++index) { code.emit("nop"); }
  const auto begin = code.pc();
  code.emit("move x:>" + number(status_slot) + ",x1");
  code.emit("move x1,sr");
  for (unsigned index = 0; index < 8; ++index) {
    code.emit("move #>" + number(word_mask) + ",m" + std::to_string(index));
  }
  code.immediate("n0", 1);
  code.immediate("r1", input_base);
  code.immediate("r2", history_base);
  code.immediate("r7", parameters);
  code.emit("move x:>" + number(entry_slot) + ",r3");
  code.emit("jsr (r3)");
  stop = code.pc();
  code.emit("nop");
  code.word(absolute_jump);
  code.word(begin);
  code.install(bus);
}
}

DspBus &Fixture::prepare(DspBus &bus, const Sources sources, unsigned &stop) {
  const auto program = sources.program;
  const auto table = sources.table;
  install_harness(bus, stop);
  bus.observe_omr(memory_mode);
  bus.advance_instruction_cycles(memory_switch_delay);
  for (std::size_t index = 0; index < program.size(); ++index) {
    bus.write({.address = program_base + static_cast<unsigned>(index), .value = program[index]});
  }
  for (std::size_t index = 0; index < table.size(); ++index) {
    bus.write({.space = DspSpace::y,
               .address = table_base + static_cast<unsigned>(index),
               .value = table[index]});
  }
  return bus;
}

Fixture::Fixture(const Sources sources)
    : program_(sources.program), table_(sources.table), cpu_(prepare(bus_, sources, stop_)) {}

void Fixture::write(const DspSpace space, const unsigned address, const unsigned value) {
  bus_.write({.space = space, .address = address, .value = value});
}

unsigned Fixture::read(const DspSpace space, const unsigned address) {
  return bus_.read({.space = space, .address = address});
}

void Fixture::seed(const Input &input, const Prediction &prediction) {
  using enum DspSpace;
  write(x,
        entry_slot,
        (input.cascade ? cascade_entries : entries).at(static_cast<unsigned>(input.kind)));
  write(x, status_slot, input.twos ? rounding_mode : 0);
  write(y, parameters, raw(input.cutoff));
  write(y, parameters + 1, raw(input.resonance));
  for (unsigned index = 0; index < 2; ++index) {
    write(x, history_base + index, initial_history.at(index));
    write(y, history_base + index, initial_history.at(index + 2));
    if (input.cascade) {
      write(x, history_base + 2 + index, initial_second_history.at(index));
      write(y, history_base + 2 + index, initial_second_history.at(index + 2));
    }
  }
  for (unsigned index = 0; index <= block_length; ++index) {
    write(x, input_base + index, prediction.input.at(index));
  }
  for (const auto space : {x, y}) {
    for (const auto address : guards(input.cascade)) { write(space, address, guard); }
    for (unsigned address = 1; address <= 3; ++address) { write(space, address, scratch_seed); }
  }
}

void Fixture::verify_history(const History &history,
                             const unsigned offset,
                             const std::string &context) {
  for (unsigned index = 0; index < 2; ++index) {
    require(read(DspSpace::x, history_base + offset + index) == history.at(index) &&
                read(DspSpace::y, history_base + offset + index) == history.at(index + 2),
            "Independent filter histories differ" + context);
  }
}

void Fixture::verify_section(const Input &input,
                             const Prediction &prediction,
                             const unsigned section) {
  const auto &coefficients = input.cascade && section == 0 ? prediction.first : prediction.final;
  const auto pc = cpu_.state().pc;
  const auto &context = prediction.context;
  require(section < (input.cascade ? 2U : 1U) &&
              pc == (coefficients.wide ? wide_core : ordinary_core),
          "Wrong independent filter core" + context);
  const unsigned offset = input.cascade && section == 1 ? 2U : 0U;
  const auto state = cpu_.state();
  require(state.r[1] == input_base && state.r[2] == history_base + offset && state.r[4] == 1 &&
              state.r[5] == input_base &&
              state.r[7] == parameters + (input.cascade && section == 0 ? 1U : 0U) &&
              state.sp == (input.cascade && section == 0 ? 2U : 1U),
          "Filter section entry frame differs" + context);
  if (input.cascade && section == 1) {
    for (unsigned index = 0; index <= block_length; ++index) {
      require(read(DspSpace::x, input_base + index) == prediction.intermediate.at(index),
              "First section output differs" + context);
    }
    verify_history(prediction.first_history, 0, context);
  }
  const std::array<unsigned, 5> actual{read(DspSpace::y, 1),
                                       read(DspSpace::y, 3),
                                       read(DspSpace::y, 2),
                                       read(DspSpace::x, 3),
                                       read(DspSpace::x, 2)};
  const std::array<unsigned, 5> expected{raw(coefficients.b0),
                                         raw(coefficients.b1),
                                         raw(coefficients.b2),
                                         raw(coefficients.a1),
                                         raw(coefficients.a2)};
  require(actual == expected, "Independently prepared filter coefficients differ" + context);
}

void Fixture::verify_result(const Input &input, const Prediction &prediction) {
  using enum DspSpace;
  const auto &coefficients = prediction.final;
  const auto &context = prediction.context;
  require(read(y, 1) == raw(coefficients.b0) && read(y, 2) == raw(coefficients.b2) &&
              read(y, 3) == raw(coefficients.b1) && read(x, 2) == raw(coefficients.a2) &&
              read(x, 3) == raw(coefficients.a1),
          "Filter coefficients changed during processing" + context);
  for (unsigned index = 0; index <= block_length; ++index) {
    require(read(x, input_base + index) == prediction.output.at(index),
            "Independent filtered sample differs at " + std::to_string(index) + context);
  }
  verify_history(prediction.first_history, 0, context);
  if (input.cascade) { verify_history(prediction.second_history, 2, context); }
  for (const auto space : {x, y}) {
    for (const auto address : guards(input.cascade)) {
      require(read(space, address) == guard, "Changed filter guard" + context);
    }
  }
  const bool writes_g = input.cascade || input.kind == Kind::low || input.kind == Kind::notch;
  require(read(x, 1) == (writes_g ? raw(coefficients.g) : scratch_seed) &&
              read(y, parameters) == raw(input.cutoff) &&
              read(y, parameters + 1) == raw(input.resonance),
          "Changed filter scratch/parameters" + context);
  const auto state = cpu_.state();
  const auto prefetch = static_cast<std::uint64_t>(2 * coefficients.b0 *
                                                   signed_word(prediction.input.at(block_length))) &
                        accumulator_mask;
  require(state.r[1] == input_base + block_length + 1 &&
              state.r[2] == history_base + (input.cascade ? 2U : 0U) && state.r[4] == 3 &&
              state.a == prefetch && (!input.cascade || state.n[1] == input_base) &&
              state.r[5] == input_base + block_length && state.r[7] == parameters &&
              state.n[0] == 1 && state.sp == 0 &&
              (state.sr & retained_status_mask) == (input.twos ? rounding_mode : 0),
          "Filter return/cursor/mode differs" + context);
}

void Fixture::run(const Input &input) {
  const auto prediction = predict(input, table_);
  seed(input, prediction);
  unsigned sections = 0;
  unsigned steps = 0;
  while (cpu_.state().pc != stop_ || steps == 0) {
    require(steps < instruction_bound, "Filter call instruction bound" + prediction.context);
    const auto pc = cpu_.state().pc;
    if (pc >= program_base && pc < program_base + program_.size()) { visited_.insert(pc); }
    if (pc == ordinary_core || pc == wide_core) { verify_section(input, prediction, sections++); }
    cpu_.step();
    bus_.advance_instruction_cycles(1);
    ++steps;
  }
  require(sections == (input.cascade ? 2U : 1U),
          "Incomplete original filter calls" + prediction.context);
  verify_result(input, prediction);
  ++counts_.calls;
  counts_.instructions += steps;
  counts_.wide += static_cast<unsigned>(prediction.final.wide);
  counts_.clipped += static_cast<unsigned>(prediction.final.clipped);
  counts_.halved += static_cast<unsigned>(prediction.final.damping_halved);
  counts_.capped += static_cast<unsigned>(prediction.final.cap_selected);
}

void Fixture::intact() {
  for (std::size_t index = 0; index < program_.size(); ++index) {
    require(bus_.peek_program(program_base + static_cast<unsigned>(index)) == program_[index],
            "Changed original filter program");
  }
  for (std::size_t index = 0; index < table_.size(); ++index) {
    require(read(DspSpace::y, table_base + static_cast<unsigned>(index)) == table_[index],
            "Changed original filter lookup data");
  }
}
}
