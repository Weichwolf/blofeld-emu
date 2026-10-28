// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_accumulator_transfer.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_middle_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::parallel_middle;

DspDebugState prepare(DspMachine &machine, const Program &code, const Case sample) {
  DspProgram image;
  image.word(move_omr);
  image.word(omr);
  // FM5.4.3.3: SZ has no defined reset value; establish the actual test precondition.
  image.word(move_sz);
  image.word(0);
  for (unsigned index = 0; index < seed_words; ++index) { image.word(code.words.at(index)); }
  image.install(machine.bus());
  for (std::size_t index = 0; index < code.words.size(); ++index) {
    machine.bus().write(
        {.address = ram + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto before = machine.cpu().debug_state();
  DspDebugState expected;
  const auto values = accumulators(sample.vector);
  expected.cpu.pc = start;
  expected.cpu.omr = omr;
  expected.cpu.sr = status | sample.flags | (sample.scale << scale_shift);
  expected.cpu.la = la_seed;
  expected.cpu.lc = lc_seed;
  expected.cpu.a = values[0];
  expected.cpu.b = values[1];
  expected.cpu.x = (std::uint64_t{x1_seed} << word_bits) | x0_seed;
  expected.cpu.y = (std::uint64_t{y1_seed} << word_bits) | y0_seed;
  expected.cpu.m.fill(word_mask);
  expected.native_instructions = seed_instructions;
  // Table counters are compared relative to the genuine preamble thereafter.
  expected.native_cycles = before.native_cycles;
  require(before == expected, "Native preamble establishes every register and all16 stack cells");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return expected;
}

struct Oracle {
  DspDebugState expected;
  unsigned next_e2 = first_e2;
  std::optional<Arithmetic> pending;
};

void retire(Oracle &oracle, const Program &code) {
  auto &expected = oracle.expected;
  const auto instruction = code.instructions.at(expected.cpu.pc - ram);
  if (instruction.effect == Effect::parallel) {
    require(oracle.pending.has_value(), "Parallel E2 retains its old-operand result from E1");
    if (oracle.pending) {
      const auto result = *oracle.pending;
      (result.b ? expected.cpu.b : expected.cpu.a) = result.value;
      expected.cpu.sr = result.status;
      oracle.pending.reset();
    }
  } else {
    require(!oracle.pending, "No fabricated ALU result for NOP or producer");
  }
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  oracle.next_e2 += 1 + code.instructions.at(expected.cpu.pc - ram).hold_before;
}

void step(DspFetchRun &run, DspMachine &machine, Oracle &oracle, const Program &code) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const bool due = cycle.completed_cycles == oracle.next_e2;
  require(phase.retirement_published == due,
          "Exact E2 placement includes only qualified source holds");
  if (due) {
    const auto pc = oracle.expected.cpu.pc;
    require(phase.execute2 && phase.execute2->address == pc &&
                phase.execute2->value == code.instructions.at(pc - ram).opcode,
            "The real instruction owns E2 retirement");
    retire(oracle, code);
  }
  const auto pc = oracle.expected.cpu.pc;
  const auto instruction = code.instructions.at(pc - ram);
  const bool write_due =
      cycle.completed_cycles + 1 == oracle.next_e2 && instruction.effect != Effect::nop;
  require(phase.transfer.has_value() == write_due, "Exact E1 publication period");
  if (write_due) {
    require(phase.transfer && phase.execute1 && phase.execute1->address == pc &&
                phase.transfer->pc == pc && phase.transfer->opcode == instruction.opcode &&
                phase.transfer->fields == fields(instruction.target),
            "Exact raw/producer lane belongs to the actual E1 instruction");
    unsigned value = x0_seed;
    if (instruction.effect == Effect::parallel) {
      const auto &cpu = oracle.expected.cpu;
      oracle.pending = arithmetic(cpu, instruction.alu);
      const auto source = instruction.source == a1_code ? cpu.a : cpu.b;
      value = static_cast<unsigned>((source >> word_bits) & word_mask);
    }
    write_word(oracle.expected.cpu, instruction.target, value);
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Full CPU,counters and16 stack cells at pc=" + hex(pc, 6) +
              " period=" + std::to_string(cycle.completed_cycles));
  require(cycle.completed_cycles < maximum_periods, "Bounded own parallel window");
}

unsigned one_case(const Case sample, const std::optional<unsigned> cut) {
  const auto code = program(sample);
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, code, sample), .pending = std::nullopt};
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const auto finish = [&] {
    while (oracle.expected.cpu.pc != code.stop) { step(run, machine, oracle, code); }
    require(!oracle.pending, "All parallel results retire through real E2");
  };
  if (!cut) {
    finish();
    return static_cast<unsigned>(run.completed_cycles());
  }
  while (run.completed_cycles() < *cut) { step(run, machine, oracle, code); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    run.restore(saved, machine);
    oracle = prefix;
    require(machine.cpu().debug_state() == oracle.expected,
            "Joint restore preserves full native state");
    finish();
  }
  return static_cast<unsigned>(run.completed_cycles());
}

void refused(const unsigned opcode) {
  require(!middle_accumulator_transfer_source(opcode),
          "Unqualified raw parallel form remains excluded");
  try {
    static_cast<void>(decode_pipeline_word(
        {.address = entry, .value = opcode}, omr, DspFetchPipeline::automatic_memory));
  } catch (const Fault &error) {
    const std::string text = error.what();
    require(text.contains("P:" + hex(entry, 6)) && text.contains("pc=" + hex(entry, 6)) &&
                text.contains("omr=" + hex(omr, 6)),
            "Refused raw parallel form retains actual Decode context");
    return;
  }
  throw Fault("Unqualified raw parallel form admitted");
}

void guards() {
  // FM Tables12-13/16: other TFR and ADD sources have no parallel raw qualification.
  constexpr std::array other_alu{0x01U,
                                 0x09U,
                                 0x41U,
                                 0x49U,
                                 0x61U,
                                 0x69U,
                                 0x71U,
                                 0x79U,
                                 0x40U,
                                 0x48U,
                                 0x50U,
                                 0x58U,
                                 0x60U,
                                 0x68U,
                                 0x70U,
                                 0x78U};
  for (const auto alu : other_alu) {
    for (const auto source : {a1_code, b1_code}) {
      for (const auto target : destinations) {
        refused(mover | (source << source_shift) | (target << target_shift) | alu);
      }
    }
  }
  constexpr std::array qualified_alu{0x51U, 0x59U, 0x10U, 0x18U};
  for (const auto alu : qualified_alu) {
    for (const auto source : {8U, 9U, 10U, 11U, 14U, 15U}) {
      for (const auto target : destinations) {
        refused(mover | (source << source_shift) | (target << target_shift) | alu);
      }
    }
    for (const auto source : {a1_code, b1_code}) {
      for (unsigned target = a0_code; target < r0_code; ++target) {
        refused(mover | (source << source_shift) | (target << target_shift) | alu);
      }
    }
  }
  std::cout << "736 parallel-middle Decode guards passed\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select streams,snapshots or guards");
    const std::string_view mode(arguments[1]);
    if (mode == "guards") {
      guards();
      return 0;
    }
    require(mode == "streams" || mode == "snapshots", "Unknown parallel-middle scope");
    unsigned checked = 0;
    unsigned periods = 0;
    unsigned selected = 0;
    for (const auto sample : cases()) {
      if (mode == "streams") {
        periods += one_case(sample, std::nullopt);
        ++checked;
      } else if (snapshot_case(sample)) {
        ++selected;
        const auto last = one_case(sample, std::nullopt);
        for (unsigned cut = 0; cut <= last; ++cut) {
          static_cast<void>(one_case(sample, cut));
          ++checked;
        }
      }
    }
    if (mode == "streams") {
      require(periods == expected_periods, "All3710 qualified periods retained");
    }
    if (mode == "snapshots") {
      require(selected == expected_snapshot_cases && checked == expected_cuts,
              "All532 period cuts across24 cases retain two independently checked suffixes");
    }
    std::cout << mode << " passed " << checked << " parallel-middle cases; selected=" << selected
              << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
