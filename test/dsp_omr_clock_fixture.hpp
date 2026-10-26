// SPDX-License-Identifier: GPL-3.0-only
#pragma once
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
#include "native_access_fixture.hpp"

#include <array>
#include <set>

namespace blofeld::test::omr_clock {
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned priority_omr = 0x600389;
constexpr unsigned body = boot + 4;
constexpr unsigned first_retirement = 7;
constexpr unsigned writer_retirement = 8;
constexpr unsigned switch_delay = 3;
constexpr unsigned x_source = 0x10000;
constexpr unsigned p_alias = 0x1000;
constexpr unsigned canary = 0x654321;
constexpr unsigned direct_prefix = 0x04c0ba;
constexpr unsigned short_prefix = 0x0500ba;
constexpr unsigned long_opcode = 0x05f43a;
constexpr unsigned short_value = 0x89;
constexpr unsigned tail = 64;

struct Source {
  unsigned field;
  unsigned seed;
};

constexpr std::array sources{Source{.field = 4, .seed = 0x44f400},
                             Source{.field = 5, .seed = 0x45f400},
                             Source{.field = 6, .seed = 0x46f400},
                             Source{.field = 7, .seed = 0x47f400},
                             Source{.field = 16, .seed = 0x60f400},
                             Source{.field = 17, .seed = 0x61f400},
                             Source{.field = 18, .seed = 0x62f400},
                             Source{.field = 19, .seed = 0x63f400},
                             Source{.field = 20, .seed = 0x64f400},
                             Source{.field = 21, .seed = 0x65f400},
                             Source{.field = 22, .seed = 0x66f400},
                             Source{.field = 23, .seed = 0x67f400}};

struct Input {
  Source source;
  unsigned value;
  unsigned opcode;
  unsigned words = 1;
};

inline DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  code.word(sr_move);
  code.word(masked_status);
  code.word(input.source.seed);
  code.word(input.value);
  code.word(0);
  code.word(input.opcode);
  if (input.words == 2) { code.word(input.value); }
  for (unsigned index = 0; index < tail; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = x_source, .value = canary});
  checkpoint::run(machine, 2);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.omr == reset_omr,
          "Actual own native seeds enter the unchanged reset mapping");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

inline DspFetchRunInputs contract() {
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  return input;
}

inline void alias(DspMachine &machine, const Input input, const unsigned completed) {
  const bool switching = (input.value & 0x80U) != 0;
  if (switching && completed >= writer_retirement + switch_delay) {
    require(machine.bus().read({.address = p_alias}) == canary,
            "Three actual subsequent periods expose preserved X storage at its real P alias");
    return;
  }
  const auto error =
      native_access::fault([&] { static_cast<void>(machine.bus().read({.address = p_alias})); });
  const bool pending = switching && completed >= writer_retirement;
  require(error.contains(pending ? "requires elapsed switch cycles" : "Unmapped DSP memory"),
          "Alias refusal distinguishes real pending delay from the original unmapped window");
}

using Receipts = std::set<unsigned>;

inline void receipts(const DspFetchCycleResult &cycle, Receipts &seen) {
  if (cycle.issued_address) {
    require(seen.insert(*cycle.issued_address).second,
            "Sequential OMR transition must not discard and refetch actual younger P words");
  }
  for (unsigned index = 0; index < cycle.fetch.used; ++index) {
    require(seen.contains(cycle.fetch.words.at(index).address),
            "Every retained P word needs its observed real F1 request");
  }
}

struct Oracle {
  DspDebugState expected;
  unsigned retired = 0;
  Receipts seen;
};

inline void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Actual native sequential order survives the raw context change");
    const bool control = oracle.retired == 1;
    const auto words = control ? input.words : 1U;
    oracle.expected.cpu.pc += words;
    ++oracle.expected.native_instructions;
    oracle.expected.native_cycles += words;
    if (control) {
      require(cycle.completed_cycles == writer_retirement,
              "Native OMR remains a one/two-unit instruction in the selected pipeline");
      oracle.expected.cpu.omr = input.value;
    }
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period matches full literal CPU/frame/PC/OMR/native-counter expectations");
  require(machine.reference_fetch().omr == oracle.expected.cpu.omr,
          "Fetch execution context changes only with its genuine native retirement");
  alias(machine, input, static_cast<unsigned>(cycle.completed_cycles));
  receipts(cycle, oracle.seen);
}

inline void one_case(const Input input) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input), .retired = 0, .seen = {}};
  DspFetchRun run(contract(), machine);
  const unsigned last = first_retirement + input.words + 3;
  while (run.completed_cycles() < last) { step(run, machine, input, oracle); }
  require(oracle.retired == 5 && run.elapsed_ticks() == last * pipeline::period,
          "Prefix NOP, control instruction and three real NOPs match independent nominal costs");
}

inline void restores() {
  const Input input{.source = sources[1], .value = priority_omr, .opcode = direct_prefix | 0x500U};
  for (unsigned cut = 4; cut <= 10; ++cut) {
    DspMachine machine;
    Oracle oracle{.expected = prepare(machine, input), .retired = 0, .seen = {}};
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix;
      while (run.completed_cycles() < 11) { step(run, machine, input, oracle); }
      require(oracle.retired == 5,
              "Trusted restore retains real OMR/F2/three-period switch ownership");
    }
  }
}
}
