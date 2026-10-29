// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_accumulator_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace blofeld::test::peripheral_accumulator {
namespace {
enum class Kind : std::uint8_t { same, other, store, clear_same, clear_other };

struct Window {
  Input input;
  Kind kind = Kind::same;
  unsigned gap = 0, cut = 0;
};

inline constexpr unsigned arithmetic_mask = 0x3e;
inline constexpr unsigned clear_flags = 0x14;

std::vector<unsigned> install(DspMachine &machine, const Window sample) {
  std::vector<std::string> code;
  const auto name = sample.input.b ? std::string("b") : std::string("a");
  const auto other = sample.input.b ? std::string("a") : std::string("b");
  const bool clear = sample.kind == Kind::clear_same || sample.kind == Kind::clear_other;
  if (clear) { code.push_back("clr " + (sample.kind == Kind::clear_same ? name : other)); }
  code.push_back(reader(sample.input));
  if (!clear) {
    code.insert(code.end(), sample.gap, "nop");
    code.push_back(sample.kind == Kind::store
                       ? "move " + name + "1,y:(r4)"
                       : "move " + (sample.kind == Kind::other ? other : name) + "1,x0");
  }
  code.insert(code.end(), 6, "nop");
  std::vector<unsigned> pcs;
  unsigned pc = entry;
  for (const auto &text : code) {
    pcs.push_back(pc);
    const auto words = checkpoint::assemble(text.c_str());
    require(words.size() == 1,
            "Own dependent window requires independently checked single-word forms");
    machine.bus().write({.address = pc++, .value = words.front()});
  }
  return pcs;
}

struct Oracle {
  DspDebugState expected;
  unsigned retired = 0, captures = 0, stalls = 0;
};

unsigned end_clock(const Window sample) {
  const bool clear = sample.kind == Kind::clear_same || sample.kind == Kind::clear_other;
  const bool stall = !clear && sample.kind != Kind::other && sample.gap == 0;
  return 14 + (clear ? 0U : sample.gap) + static_cast<unsigned>(stall);
}

void retire_expected(Oracle &oracle, const Window sample, const unsigned address) {
  const bool clear = sample.kind == Kind::clear_same || sample.kind == Kind::clear_other;
  const auto offset = address - entry;
  if (clear && offset == 0) {
    const bool b = sample.input.b != (sample.kind == Kind::clear_other);
    (b ? oracle.expected.cpu.b : oracle.expected.cpu.a) = 0;
    oracle.expected.cpu.sr = (oracle.expected.cpu.sr & ~arithmetic_mask) | clear_flags;
  }
  if (!clear && offset == 1 + sample.gap && sample.kind != Kind::store) {
    const auto value = sample.kind == Kind::other
                           ? ((sample.input.b ? seed_a : seed_b) >> word_bits) & word_mask
                           : source_value(sample.input);
    oracle.expected.cpu.x = (oracle.expected.cpu.x & ~std::uint64_t{word_mask}) | value;
  }
  oracle.expected.cpu.pc = address + 1;
  ++oracle.expected.native_instructions;
  ++oracle.expected.native_cycles;
  ++oracle.retired;
}

void advance(DspFetchRun &run,
             DspMachine &machine,
             const Window sample,
             const std::vector<unsigned> &pcs,
             Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  if (cycle.bus.peripheral_read) { ++oracle.captures; }
  if (phase.memory_source_stall) {
    ++oracle.stalls;
    require(sample.kind != Kind::other && sample.kind != Kind::clear_same &&
                sample.kind != Kind::clear_other && sample.gap == 0 && cycle.completed_cycles == 7,
            "Only the adjacent real accumulator bus consumer interlocks for one period");
  }
  // E2 publishes older CPU/CCR first; the same period's newer native E1 load wins overlapping
  // fields.
  if (phase.retirement_published) {
    if (!phase.execute2) { throw Fault("Native dependent retirement lost its real E2 word"); }
    require(oracle.retired < pcs.size() && phase.execute2->address == pcs.at(oracle.retired),
            "Every dependent-window retirement follows the independent own instruction path");
    retire_expected(oracle, sample, phase.execute2->address);
  }
  const bool clear = sample.kind == Kind::clear_same || sample.kind == Kind::clear_other;
  if (cycle.completed_cycles == (clear ? 7U : 6U)) { load_result(oracle.expected, sample.input); }
  if (!clear && sample.kind != Kind::store && phase.transfer &&
      phase.transfer->pc == entry + 1 + sample.gap) {
    const auto value = sample.kind == Kind::other
                           ? ((sample.input.b ? seed_a : seed_b) >> word_bits) & word_mask
                           : source_value(sample.input);
    oracle.expected.cpu.x = (oracle.expected.cpu.x & ~std::uint64_t{word_mask}) | value;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every dependency period preserves independent full accumulator/CCR/CPU/counter results");
}

void finish(DspFetchRun &run,
            DspMachine &machine,
            const Window sample,
            const std::vector<unsigned> &pcs,
            Oracle &oracle) {
  while (run.completed_cycles() < end_clock(sample)) { advance(run, machine, sample, pcs, oracle); }
  const bool clear = sample.kind == Kind::clear_same || sample.kind == Kind::clear_other;
  const unsigned stalls = !clear && sample.kind != Kind::other && sample.gap == 0 ? 1U : 0U;
  require(
      oracle.captures == 1 && oracle.stalls == stalls && oracle.retired == pcs.size(),
      "Independent model windows require exactly one sampled read and the declared consumer stall");
  require(
      machine.bus().read({.space = DspSpace::y, .address = source_data}) ==
          (sample.kind == Kind::store ? source_value(sample.input) : canary),
      "The native memory consumer stores the actual new A1/B1 without a host data substitution");
}

void window(const Window sample) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, sample.input)};
  const auto pcs = install(machine, sample);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  if (sample.cut == 0) {
    finish(run, machine, sample, pcs, oracle);
    return;
  }
  while (run.completed_cycles() < sample.cut) { advance(run, machine, sample, pcs, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    oracle = prefix;
    finish(run, machine, sample, pcs, oracle);
  }
}

void consumers(const Input input) {
  for (const auto kind : {Kind::same, Kind::other, Kind::store}) {
    for (unsigned gap = 0; gap < 3; ++gap) {
      window({.input = input, .kind = kind, .gap = gap});
      if (gap != 0 || (input.value != positive_edge && input.value != negative_edge)) { continue; }
      for (unsigned cut = 7; cut <= 8; ++cut) {
        window({.input = input, .kind = kind, .cut = cut});
      }
    }
  }
}

void preceding_alu(const Input input) {
  for (const auto kind : {Kind::clear_same, Kind::clear_other}) {
    window({.input = input, .kind = kind});
    if (input.value != positive_edge && input.value != negative_edge) { continue; }
    for (unsigned cut = 5; cut <= 8; ++cut) { window({.input = input, .kind = kind, .cut = cut}); }
  }
}
}

void dependency_checks() {
  for (unsigned port = 0; port < 2; ++port) {
    for (const bool b : {false, true}) {
      for (const auto value : values) {
        consumers({.port = port, .value = value, .status = masked_status | all_ccr, .b = b});
        preceding_alu({.port = port, .value = value, .status = masked_status | 3U, .b = b});
      }
    }
  }
  std::cout << "352 consumer/preceding-ALU windows;112 cuts/224 independent suffixes passed\n";
}
}
