// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
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
constexpr unsigned entry = 0x100;
constexpr unsigned body = entry + 1;
constexpr unsigned operand = 0x407;
constexpr unsigned other_operand = 0x409;
constexpr unsigned padding = 16;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned word_bits = 24;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned sign_word = 0x800000;
constexpr unsigned canary = 0x654321;
constexpr unsigned illegal_word = 7;
constexpr std::uint64_t period = 45;
constexpr unsigned offset_register_begin = 24;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned compatible_address_mask = 0xffff;
constexpr unsigned sc_settle_cycles = 3;
constexpr unsigned last_clock = 9;
constexpr unsigned r2_field = 18;
constexpr unsigned field_high_mask = 0x18;
constexpr unsigned field_high_shift = 17;
constexpr std::array data_seeds{0x234567U, 0x800001U, 0x765432U, 0xffffffU};
constexpr std::uint64_t sign_extension = 0xff000000000000;
constexpr std::array values{0U, 0x7fffffU, 0x800000U, 0xffffffU};
constexpr std::array flags{0x55U, 0xaaU};
// FM12-14 X/Y memory register fields;14/15 denote full A/B.
constexpr std::array destinations{4U, 5U, 6U, 7U, 16U, 18U, 23U, 24U, 31U};
// FM12-16/13-178: opposite accumulator, X0,Y0,X1,Y1.
constexpr std::array source_fields{0U, 4U, 5U, 6U, 7U};
constexpr std::array source_names{"", "x0", "y0", "x1", "y1"};

struct Input {
  unsigned source = 0;
  unsigned destination = 0;
  unsigned variant = 0;
  unsigned status = 0;
  bool b = false;
  bool y = false;
  bool compatibility = false;
};

unsigned opcode(const Input sample) {
  constexpr unsigned scalar_base = 0x400000;
  constexpr unsigned absolute_load = 0xf000;
  constexpr unsigned y_bit = 0x80000;
  constexpr unsigned tfr_base = 1;
  constexpr unsigned b_bit = 8;
  const auto reg = sample.destination;
  return scalar_base | ((reg & field_high_mask) << field_high_shift) | ((reg & 7U) << 16U) |
         absolute_load | (sample.y ? y_bit : 0U) | tfr_base |
         (source_fields.at(sample.source) << 4U) | (sample.b ? b_bit : 0U);
}

std::string register_name(const unsigned field) {
  if (field < 8) { return std::array{"x0", "x1", "y0", "y1"}.at(field - 4); }
  if (field < 14) { return std::array{"a0", "b0", "a2", "b2", "a1", "b1"}.at(field - 8); }
  if (field == 14 || field == 15) { return field == 14 ? "a" : "b"; }
  return std::string(field < offset_register_begin ? "r" : "n") + std::to_string(field & 7U);
}

std::uint64_t extended(const unsigned value) {
  return (std::uint64_t{value} << word_bits) | ((value & sign_word) != 0 ? sign_extension : 0U);
}

std::uint64_t source(const DspState &state, const Input sample) {
  switch (sample.source) {
    case 0: return sample.b ? state.a : state.b;
    case 1: return extended(static_cast<unsigned>(state.x & word_mask));
    case 2: return extended(static_cast<unsigned>(state.y & word_mask));
    case 3: return extended(static_cast<unsigned>((state.x >> word_bits) & word_mask));
    case 4: return extended(static_cast<unsigned>((state.y >> word_bits) & word_mask));
    default: throw Fault("Invalid own TFR source");
  }
}

void assign(DspState &state, const Input sample) {
  const unsigned field = sample.destination;
  const unsigned value = values.at(sample.variant);
  const unsigned address_value =
      value & (sample.compatibility ? compatible_address_mask : word_mask);
  if (field >= offset_register_begin) {
    state.n.at(field & 7U) = address_value;
  } else if (field >= 16) {
    state.r.at(field & 7U) = address_value;
  } else if (field == 14 || field == 15) {
    (field == 14 ? state.a : state.b) = extended(value);
  } else if (field >= 8) {
    // FM Table12-14: A0/B0,A2/B2,A1/B1 use only the selected raw part.
    constexpr std::array shifts{0U, 2U * word_bits, word_bits};
    const auto part = (field - 8U) / 2U;
    const auto shift = shifts.at(part);
    const std::uint64_t mask = part == 1 ? byte_mask : word_mask;
    auto &accumulator = (field & 1U) == 0 ? state.a : state.b;
    accumulator = (accumulator & ~(mask << shift)) | ((value & mask) << shift);
  } else {
    auto &pair = field < 6 ? state.x : state.y;
    const unsigned shift = (field & 1U) * word_bits;
    pair = (pair & ~(std::uint64_t{word_mask} << shift)) | (std::uint64_t{value} << shift);
  }
}

DspFetchRunInputs contract() {
  auto result = test::fetch_clock::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  result.execution = DspReferenceExecution::staged_register_boundaries;
  return result;
}

void encoding(const Input sample) {
  const auto src =
      sample.source == 0 ? std::string(sample.b ? "a" : "b") : source_names.at(sample.source);
  const auto text = "tfr " + src + ',' + (sample.b ? "b" : "a") + (sample.y ? " y:>" : " x:>") +
                    test::number(operand) + ',' + register_name(sample.destination);
  const auto words = test::checkpoint::assemble(text.c_str());
  require(words == std::vector<std::uint32_t>{opcode(sample), operand},
          "Own assembly must match independent FM register/source/space/address bit fields");
}

unsigned status(const Input sample) {
  return test::masked_status | sample.status | (sample.compatibility ? test::compatibility : 0U);
}

DspDebugState prepare(DspMachine &machine, const Input sample, const bool reference = true) {
  test::DspProgram code;
  code.immediate("omr", ordinary_omr);
  for (unsigned i = 0; i < 8; ++i) {
    code.immediate("r" + std::to_string(i), canary + i);
    code.immediate("n" + std::to_string(i), i + 1);
    code.immediate("m" + std::to_string(i), word_mask);
  }
  constexpr std::array names{"x0", "x1", "y0", "y1"};
  for (unsigned i = 0; i < names.size(); ++i) { code.immediate(names.at(i), data_seeds.at(i)); }
  constexpr std::array<std::uint64_t, 2> accumulators{0x7f765432123456, 0x80876543654321};
  for (unsigned accumulator = 0; accumulator < accumulators.size(); ++accumulator) {
    for (unsigned part = 0; part < 3; ++part) {
      code.immediate(std::string(accumulator == 0 ? "a" : "b") + std::to_string(part),
                     (accumulators.at(accumulator) >> (part * word_bits)) &
                         (part == 2 ? byte_mask : word_mask));
    }
  }
  code.immediate("sr", status(sample));
  if (sample.compatibility) {
    // FM4-3 and Table5-1: three NOPs establish the changed SC mode.
    for (unsigned i = 0; i < sc_settle_cycles; ++i) { code.word(0); }
  }
  code.word(absolute_jump);
  code.word(entry);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned pc = entry; pc < entry + padding; ++pc) { machine.bus().write({.address = pc}); }
  machine.bus().write({.address = body, .value = opcode(sample)});
  machine.bus().write({.address = body + 1, .value = operand});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {operand - 1, operand, operand + 1, other_operand}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  machine.bus().write({.space = sample.y ? DspSpace::y : DspSpace::x,
                       .address = operand,
                       .value = values.at(sample.variant)});
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.sr == status(sample),
          "Native seed instructions establish the own stream and literal CCR");
  if (reference) { static_cast<void>(machine.begin_reference_instruction(contract().execution)); }
  return before;
}

DspDebugState expected(const DspDebugState &before, const Input sample, const unsigned clock) {
  auto result = before;
  if (clock >= 7) {
    assign(result.cpu, sample);
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  if (clock >= 8) {
    (sample.b ? result.cpu.b : result.cpu.a) = source(before.cpu, sample);
    result.cpu.pc += 2;
    ++result.native_instructions;
    result.native_cycles += 2;
  }
  return result;
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Input sample) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Own parallel absolute stream lost its pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const unsigned fetch_accesses = clock == 1 ? 0U : 1U;
  const unsigned data_accesses = clock == 7 ? 1U : 0U;
  require(cycle.bus.core_access_count == fetch_accesses + data_accesses,
          "Every captured F2 word and the single real data read must remain in the footprint");
  const DspProgramWord word{.address = body, .value = opcode(sample)};
  const DspProgramWord extension{.address = body + 1, .value = operand};
  if (clock == 4) {
    require(phase.decode_accepted && phase.decode == word && phase.decode_words == 2,
            "Two-word parallel form must use its actual captured F2 occurrence");
    machine.bus().write({.address = body, .value = illegal_word});
    machine.bus().write({.address = body + 1, .value = other_operand});
  }
  if (clock == 5) {
    require(phase.ag1 == word && phase.ag1_extension == extension &&
                phase.extension_decode == extension,
            "AG1 must retain the original long-absolute extension after backing memory changes");
  }
  if (clock == 7) {
    require(phase.execute1 == word && phase.e1_extension == extension && phase.memory &&
                phase.memory->address.kind == DspAddressStageKind::absolute_long &&
                !phase.memory->address.pointer_update &&
                phase.memory->data.kind == DspNativeAccessKind::read &&
                phase.memory->data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                phase.memory->data.target.address == operand &&
                phase.memory->data.value == values.at(sample.variant),
            "One actual E1 memory read uses the captured address and independent payload");
  }
  require(phase.memory.has_value() == (clock == 7) && phase.agu_dependencies == 0 &&
              !phase.memory_source_stall &&
              phase.retirement_published == (clock == 7 || clock == 8),
          "Literal addressing has no invented AGU input/stall or duplicate data/native retirement");
  require(
      machine.cpu().debug_state() == expected(before, sample, clock),
      "E1 move/E2 TFR and full unchanged CCR/stack/RNM follow independent literal expectations");
  require(run.elapsed_ticks() == clock * period,
          "Every elapsed period remains in the exact ledger");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {operand - 1, operand, operand + 1, other_operand}) {
      const unsigned value = address == operand && (space == DspSpace::y) == sample.y
                                 ? values.at(sample.variant)
                                 : canary;
      require(machine.bus().read({.space = space, .address = address}) == value,
              "Source,neighbor and opposite-space cells must remain unchanged");
    }
  }
}

void finish(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const Input sample) {
  while (run.completed_cycles() < last_clock) { step(run, machine, before, sample); }
}

unsigned windows = 0;
unsigned restored = 0;
unsigned clones = 0;
unsigned compatibility_windows = 0;

void one(const Input sample) {
  encoding(sample);
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(contract(), machine);
  std::vector<DspFetchCheckpoint> cuts;
  const bool snapshots = (sample.source == 3 && sample.destination == r2_field &&
                          sample.variant == 2 && sample.status == flags.at(1)) ||
                         (sample.source == 0 && sample.destination == (sample.b ? 8U : 9U) &&
                          sample.variant == 1 && sample.status == flags.at(0));
  while (run.completed_cycles() < last_clock) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    step(run, machine, before, sample);
  }
  for (const auto &cut : cuts) {
    run.restore(cut, machine);
    finish(run, machine, before, sample);
    ++restored;
    DspMachine target;
    DspFetchRun clone(cut, target);
    finish(clone, target, before, sample);
    require(
        target.cpu().debug_state() == machine.cpu().debug_state(),
        "Fresh ownership must retain actual captured words and every native register/stack field");
    ++clones;
  }
  ++windows;
}

void compatibility_one(const Input sample) {
  encoding(sample);
  DspMachine machine;
  const auto before = prepare(machine, sample, false);
  auto nop = before;
  ++nop.cpu.pc;
  ++nop.native_instructions;
  ++nop.native_cycles;
  machine.cpu().step();
  require(machine.cpu().debug_state() == nop,
          "Stable SC=1 native NOP preserves full state before the parallel instruction");
  machine.cpu().step();
  const auto actual = machine.cpu().debug_state();
  const auto wanted = expected(before, sample, 8);
  if (actual != wanted) {
    std::cerr << "SC1 source=" << sample.source << " destination=" << sample.destination
              << " variant=" << sample.variant << " flags=" << sample.status << " b=" << sample.b
              << " y=" << sample.y << " pc=" << hex(actual.cpu.pc, 6) << '/'
              << hex(wanted.cpu.pc, 6) << " sr=" << hex(actual.cpu.sr, 6) << '/'
              << hex(wanted.cpu.sr, 6) << " a=" << hex(actual.cpu.a, 14) << '/'
              << hex(wanted.cpu.a, 14) << " b=" << hex(actual.cpu.b, 14) << '/'
              << hex(wanted.cpu.b, 14) << " x=" << hex(actual.cpu.x, 12) << '/'
              << hex(wanted.cpu.x, 12) << " y=" << hex(actual.cpu.y, 12) << '/'
              << hex(wanted.cpu.y, 12) << " instructions=" << actual.native_instructions << '/'
              << wanted.native_instructions << " cycles=" << actual.native_cycles << '/'
              << wanted.native_cycles << '\n';
    for (unsigned i = 0; i < actual.cpu.r.size(); ++i) {
      if (actual.cpu.r.at(i) != wanted.cpu.r.at(i) || actual.cpu.n.at(i) != wanted.cpu.n.at(i)) {
        std::cerr << "AGU " << i << " r=" << hex(actual.cpu.r.at(i), 6) << '/'
                  << hex(wanted.cpu.r.at(i), 6) << " n=" << hex(actual.cpu.n.at(i), 6) << '/'
                  << hex(wanted.cpu.n.at(i), 6) << '\n';
      }
    }
  }
  require(actual == wanted,
          "Native stable SC=1 narrows only AGU load destinations and retains old-source TFR");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {operand - 1, operand, operand + 1, other_operand}) {
      const unsigned value = address == operand && (space == DspSpace::y) == sample.y
                                 ? values.at(sample.variant)
                                 : canary;
      require(machine.bus().read({.space = space, .address = address}) == value,
              "Native SC=1 read retains source, neighbor and opposite-space cells");
    }
  }
  ++compatibility_windows;
}

void execute(const Input sample) {
  if (sample.compatibility) {
    compatibility_one(sample);
  } else {
    one(sample);
  }
}

void variants(Input sample) {
  for (unsigned variant = 0; variant < values.size(); ++variant) {
    for (const auto status : flags) {
      sample.variant = variant;
      sample.status = status;
      for (const auto destination : destinations) {
        sample.destination = destination;
        execute(sample);
      }
      sample.destination = sample.b ? 14U : 15U;
      execute(sample);
      for (const unsigned partial : {8U, 10U, 12U}) {
        sample.destination = partial + (sample.b ? 0U : 1U);
        execute(sample);
      }
    }
  }
}

void matrix(const bool compatibility) {
  for (unsigned source = 0; source < source_fields.size(); ++source) {
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        variants({.source = source, .b = b, .y = y, .compatibility = compatibility});
      }
    }
  }
}

void guards() {
  const Input sample{.source = 3, .destination = r2_field, .status = flags.at(0)};
  constexpr unsigned read_bit = 0x8000;
  require(!decode_memory_stage(opcode(sample) & ~read_bit),
          "Parallel TFR stores remain outside this read-only qualification");
  constexpr unsigned alu_mask = 0xff;
  constexpr unsigned unsupported_add = 0x40;
  require(!decode_memory_stage((opcode(sample) & ~alu_mask) | unsupported_add),
          "Other parallel ALU operations require separate qualification");
  DspMachine machine;
  Input overlap = sample;
  overlap.destination = 14; // Full A conflicts with TFR's A result.
  const auto before = prepare(machine, overlap);
  DspFetchRun run(contract(), machine);
  std::string fault;
  try {
    finish(run, machine, before, overlap);
  } catch (const Fault &error) { fault = error.what(); }
  require(fault.contains("overlapping native ALU/move-writeback accumulator") &&
              machine.cpu().debug_state() == before,
          "Overlapping accumulator destinations must reject before native or memory effects");
  DspMachine compatibility;
  Input compatible = sample;
  compatible.compatibility = true;
  const auto compatible_before = prepare(compatibility, compatible);
  const auto refusal = test::checkpoint::rejects(
      [&] { const DspFetchRun rejected(contract(), compatibility); },
      "Automatic fetch must retain its explicit unqualified SC=1 refusal");
  require(refusal.contains("Program-fetch start requires an ordinary pending opcode") &&
              compatibility.cpu().debug_state() == compatible_before,
          "Unqualified automatic SC=1 start remains a visible refusal before instruction effects");
  std::cout << "PARALLEL_ABSOLUTE guards=4 passed\n";
}

}

int main() {
  try {
    std::cout << "PARALLEL_ABSOLUTE begin primary-matrix\n" << std::flush;
    matrix(false);
    std::cout << "PARALLEL_ABSOLUTE pipeline-windows=" << windows << " restores=" << restored
              << " clones=" << clones << " passed\n"
              << std::flush;
    matrix(true);
    guards();
    std::cout << "PARALLEL_ABSOLUTE compatibility-windows=" << compatibility_windows << " passed\n";
    std::cout << "PARALLEL_ABSOLUTE windows=" << windows << " restores=" << restored
              << " clones=" << clones << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
