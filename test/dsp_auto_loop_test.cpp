// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned guard_words = 4;
constexpr unsigned setup = pipeline::body + guard_words;
constexpr unsigned first = setup + 2;
constexpr unsigned decode_clock = 3 + guard_words;
constexpr unsigned maximum_clocks = 100;
constexpr unsigned immediate_do = 0x060080;

struct Case {
  unsigned count;
  unsigned size;

  [[nodiscard]] unsigned after() const { return first + size; }

  [[nodiscard]] unsigned native_count() const { return guard_words + 1 + (count * size); }

  [[nodiscard]] unsigned setup_delay() const {
    if (count == 0) { return 8; }
    return size == 1 ? 6U : 5U;
  }
};

struct Observation {
  unsigned retired = 0;
  unsigned body_decodes = 0;
  unsigned target_decode = 0;
};

void decode(const DspPipelineCycle &phase,
            const unsigned clock,
            const Case sample,
            Observation &observed) {
  if (!phase.decode_accepted || !phase.decode) { return; }
  const auto pc = phase.decode->address;
  if (pc < setup) { return; }
  if (pc == setup) {
    require(clock == decode_clock,
            "DO enters actual Decode after the literal four-NOP guard prefix");
  } else if (pc < sample.after()) {
    const auto expected_pc = first + (observed.body_decodes % sample.size);
    require(pc == expected_pc, "Repeated Decode follows literal body and iteration order");
    if (observed.body_decodes == 0) { observed.target_decode = clock; }
    ++observed.body_decodes;
  } else if (sample.count == 0 && observed.target_decode == 0) {
    observed.target_decode = clock;
    require(pc == sample.after(), "Zero DO selects actual Decode after its annulled body");
  }
}

void retired(const DspMachine &machine, const Case sample, const Observation observed) {
  if (observed.retired <= guard_words) {
    require(machine.cpu().state().pc == pipeline::body + observed.retired,
            "The preceding four native NOPs retire at literal prefix addresses");
    return;
  }
  const auto executed = observed.retired - guard_words - 1;
  const bool done = observed.retired == sample.native_count();
  const auto expected_pc = done ? sample.after() : first + (executed % sample.size);
  require(machine.cpu().state().pc == expected_pc,
          "Actual native E2 follows independent DO/NOP/backedge/fallthrough PC");
}

void final_state(const DspMachine &machine, const DspDebugState &before, const Case sample) {
  auto expected = before;
  expected.cpu.pc = sample.after();
  expected.stack_high.at(1) = before.cpu.la;
  expected.stack_low.at(1) = before.cpu.lc;
  expected.stack_high.at(2) = first;
  expected.stack_low.at(2) = before.cpu.sr;
  const auto actual = machine.cpu().debug_state();
  require(
      actual.cpu == expected.cpu && actual.stack_high == expected.stack_high &&
          actual.stack_low == expected.stack_low && actual.sc == expected.sc &&
          actual.native_instructions == before.native_instructions + sample.native_count(),
      "All CPU fields, literal DO frame cells and native retirement count match independent state");
}

struct PhaseTrace {
  std::vector<unsigned> pcs;
  std::vector<unsigned> decode_clocks;
  unsigned executed = 0;

  void check(const DspPipelineCycle &phase, const unsigned clock, const unsigned retired_count) {
    if (phase.decode_accepted && phase.decode) {
      pcs.push_back(phase.decode->address);
      decode_clocks.push_back(clock);
    }
    if (phase.execute1) {
      require(executed < pcs.size() && phase.execute1->address == pcs.at(executed) &&
                  clock == decode_clocks.at(executed) + 3 && phase.execution_captured,
              "Every real native E1 follows its exact prior Decode occurrence by three clocks");
      ++executed;
    }
    if (phase.execute2) {
      require(retired_count < pcs.size() && phase.execute2->address == pcs.at(retired_count) &&
                  clock == decode_clocks.at(retired_count) + 4 && phase.retirement_published,
              "Every real native E2 publishes its exact prior Decode occurrence four clocks later");
    }
  }
};

void target_fetch(const DspFetchCycleResult &cycle, const unsigned clock, const Case sample) {
  const auto target = sample.count == 0 ? sample.after() : first;
  if (clock == decode_clock + 3) {
    require(cycle.issued_address == target,
            "Actual native DO E1 selects exactly one first body/annul F1 address");
  }
  if (clock == decode_clock + 4) {
    require(cycle.bus.fetch && cycle.bus.fetch->used != 0 &&
                cycle.bus.fetch->words.at(cycle.bus.fetch->used - 1).address == target,
            "Actual selected body/annul F2 arrives on native DO E2");
  }
  if (sample.count != 0 || !cycle.bus.fetch) { return; }
  for (unsigned index = 0; index < cycle.bus.fetch->used; ++index) {
    const auto pc = cycle.bus.fetch->words.at(index).address;
    require(pc < first || pc >= sample.after(), "Zero DO never reads an annulled body P word");
  }
}

void run_case(const Case sample) {
  const auto text = "do #" + std::to_string(sample.count) + "," + number(sample.after());
  const auto encoded = checkpoint::assemble(text.c_str());
  require(encoded.size() == 2 && encoded.front() == (immediate_do | (sample.count << 8)) &&
              encoded.back() == sample.after() - 1,
          "Installed DO words agree with independent literal fields and last address");
  std::vector<const char *> words(guard_words, "nop");
  words.push_back(text.c_str());
  words.insert(words.end(), sample.size, "nop");
  DspMachine machine;
  pipeline::prepare(machine, words);
  const auto before = machine.cpu().debug_state();
  auto inputs = pipeline::contract();
  inputs.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(inputs, machine);
  Observation observed;
  PhaseTrace trace;
  for (unsigned clock = 1; clock < maximum_clocks; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    trace.check(phase, clock, observed.retired);
    target_fetch(cycle, clock, sample);
    decode(phase, clock, sample, observed);
    if (phase.retirement_published) {
      ++observed.retired;
      retired(machine, sample, observed);
    }
    require(cycle.bus.core_access_count <= 1,
            "Every elapsed period has at most one actual P read in this memory-free stream");
    if (clock > decode_clock && clock < decode_clock + 3) {
      require(!cycle.issued_address, "No body F1 before actual DO E1 selects its target");
    }
    if (observed.retired == sample.native_count() && observed.target_decode != 0) { break; }
  }
  require(observed.retired == sample.native_count() &&
              observed.target_decode == decode_clock + sample.setup_delay(),
          "Actual setup timing matches independent normal/degenerate/zero Decode spacing");
  final_state(machine, before, sample);
}
}

int main() {
  try {
    for (const unsigned count : std::array{0U, 1U, 2U, 4U}) {
      for (const unsigned size : std::array{1U, 2U, 5U}) {
        run_case({.count = count, .size = size});
      }
    }
    std::cout
        << "Automatic native counted DO setup, actual F2 and zero-overhead NOP loop cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
