// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_mpy_recurrence_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test {
namespace {
constexpr unsigned entry = 0x100, base = 0x400, ring = 8, stride = 3, terms = 7, tail = 16;
constexpr unsigned status = 0xc00300, normal_omr = 0x300, modifier = 0x8007;
constexpr unsigned coefficient = 0x200000, seed = 0x400000, minus_one = 0x800000;
constexpr unsigned seed_instructions = 12, operation_count = 16, bound = 200, canary = 0x123456;
constexpr unsigned growth_bit = 46;
constexpr unsigned jump = 0x0af080;
constexpr std::int64_t span = 0x1000000, half = 0x800000;
constexpr std::int64_t min_word = -0x800000, max_word = 0x7fffff;

std::int64_t rounded(const std::int64_t value) {
  auto quotient = value / span;
  auto remainder = value % span;
  if (remainder < 0) {
    --quotient;
    remainder += span;
  }
  if (remainder > half || (remainder == half && quotient % 2 != 0)) { ++quotient; }
  return quotient * span;
}

struct Expected {
  DspDebugState cpu;
  std::array<unsigned, ring> words{};
};

Expected expected_ring(const DspDebugState &before) {
  Expected result{.cpu = before};
  std::int64_t accumulator = seed * span;
  std::int64_t previous = 0;
  unsigned flags = 0;
  for (unsigned term = 1; term <= terms; ++term) {
    const auto raw = static_cast<std::uint64_t>(accumulator) & accumulator_mask;
    if ((((raw >> growth_bit) ^ (raw >> (growth_bit - 1))) & 1U) != 0) { flags |= sticky; }
    const auto floor = accumulator >= 0 ? accumulator / span : -((-accumulator + span - 1) / span);
    const auto current = std::clamp(floor, min_word, max_word);
    if (current != floor) { flags |= limit; }
    result.words.at((stride * term) % ring) = static_cast<unsigned>(current) & word_mask;
    accumulator =
        rounded((-previous * span) + (2 * static_cast<std::int64_t>(coefficient) * current));
    previous = current;
  }
  result.cpu.cpu.a = static_cast<std::uint64_t>(accumulator) & accumulator_mask;
  result.cpu.cpu.y = static_cast<unsigned>(previous) & word_mask;
  result.cpu.cpu.r[4] = base;
  result.cpu.cpu.pc = entry + operation_count;
  result.cpu.cpu.sr = status | flags | value_flags({.value = result.cpu.cpu.a, .status = status});
  result.cpu.native_instructions += operation_count;
  result.cpu.native_cycles += operation_count;
  return result;
}

DspDebugState prepare(DspMachine &machine) {
  DspProgram code;
  code.immediate("omr", normal_omr);
  code.immediate("sr", status);
  code.immediate("x0", minus_one);
  code.immediate("x1", coefficient);
  code.immediate("y0", 0);
  code.immediate("a0", 0);
  code.immediate("a1", seed);
  code.immediate("a2", 0);
  code.immediate("r4", base);
  code.immediate("n4", stride);
  code.immediate("m4", modifier);
  code.word(jump);
  code.word(entry);
  code.install(machine.bus());
  std::array<unsigned, operation_count + tail> words{};
  const auto store = checkpoint::assemble("move y0,y:(r4)+n4");
  const auto mpy = checkpoint::assemble("mpy y0,x0,a a,y0");
  const auto macr = checkpoint::assemble("macr x1,y0,a y0,y:(r4)+n4");
  require(store.size() == 1 && mpy.size() == 1 && macr.size() == 1,
          "Own recurrence uses three one-word ISA forms");
  words[0] = store[0];
  for (unsigned term = 0; term < terms; ++term) {
    words.at(1 + (2 * term)) = mpy[0];
    words.at(2 + (2 * term)) = macr[0];
  }
  for (unsigned index = 0; index < words.size(); ++index) {
    machine.bus().write({.address = entry + index, .value = words.at(index)});
  }
  for (unsigned address = base - 1; address <= base + ring; ++address) {
    machine.bus().write({.space = DspSpace::y, .address = address, .value = canary});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.a == static_cast<std::uint64_t>(seed) * span &&
              before.cpu.x == (std::uint64_t{coefficient} << word_bits | minus_one) &&
              before.cpu.y == 0 && before.cpu.sr == status && before.cpu.r[4] == base &&
              before.cpu.n[4] == stride && before.cpu.m[4] == modifier,
          "Actual native setup establishes independent recurrence and multiple-wrap inputs");
  return before;
}

bool advance(DspMachine &machine, DspFetchRun &run) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  return phase.retirement_published && phase.execute2 &&
         phase.execute2->address == entry + operation_count - 1;
}

void finish(DspMachine &machine, DspFetchRun &run, const Expected &expected) {
  bool done = false;
  for (unsigned i = 0; i < bound && !done; ++i) { done = advance(machine, run); }
  require(done && machine.cpu().debug_state() == expected.cpu,
          "Independent full recurrence endpoint,flags,stack,native costs and ring cursor");
  for (unsigned i = 0; i < ring; ++i) {
    require(machine.bus().read({.space = DspSpace::y, .address = base + i}) == expected.words.at(i),
            "Seven dependent MACR results and initial zero retain independent permuted RAM words");
  }
  for (const unsigned address : {base - 1, base + ring}) {
    require(machine.bus().read({.space = DspSpace::y, .address = address}) == canary,
            "Ring wrap preserves adjacent canaries");
  }
}
}

void mpy_recurrence() {
  DspMachine machine;
  const auto expected = expected_ring(prepare(machine));
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(input, machine);
  pipeline::clocks(run, machine, 7);
  const auto saved = run.checkpoint(machine);
  finish(machine, run, expected);
  const auto ticks = run.elapsed_ticks();
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    finish(machine, run, expected);
    require(run.elapsed_ticks() == ticks, "Dependent recurrence restores original elapsed clock");
  }
  DspMachine target;
  DspFetchRun clone(saved, target);
  finish(target, clone, expected);
  require(clone.elapsed_ticks() == ticks,
          "Fresh recurrence owner retains old-A transfer and pending multiply");
}
}
