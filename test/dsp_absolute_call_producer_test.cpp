// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_control_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned indirect_call = 0x0be080; // FM13-89: JSR direct Rn.
constexpr unsigned frame_flags = 0xaa;
constexpr unsigned body_start = boot + 4;
constexpr unsigned move_r = 0x60f400;
constexpr unsigned move_n = 0x70f400;
constexpr unsigned move_m = 0x05f420;

enum class Producer : std::uint8_t { none, r, n, m };
enum class Target : std::uint8_t { forward, sequential, self };

struct Case {
  unsigned reg = 0;
  Producer producer = Producer::none;
  unsigned gap = 0;
  Target target = Target::forward;
  unsigned flags = 0;
};

struct Program {
  unsigned jump;
  unsigned target;
  std::vector<std::uint32_t> seeds;
  std::vector<std::uint32_t> words;
};

Program program(const Case sample) {
  const unsigned producer_words = sample.producer == Producer::none ? 0U : 2U;
  const unsigned jump = body_start + 1 + producer_words + sample.gap;
  unsigned target = jump + 13;
  if (sample.target == Target::self) {
    target = jump;
  } else if (sample.target == Target::sequential) {
    target = jump + 1;
  }
  const unsigned old_r = sample.producer == Producer::r ? target + 1 : target;
  Program result{.jump = jump,
                 .target = target,
                 .seeds = {control::sr_move,
                           masked_status | sample.flags,
                           move_r | (sample.reg << 16U),
                           old_r},
                 .words = std::vector<std::uint32_t>(control::padding, 0)};
  if (sample.producer != Producer::none) {
    unsigned opcode = move_r | (sample.reg << 16U);
    unsigned value = target;
    if (sample.producer == Producer::n) {
      opcode = move_n | (sample.reg << 16U);
      value = 4;
    } else if (sample.producer == Producer::m) {
      opcode = move_m | sample.reg;
      value = word_mask;
    }
    result.words.at(1) = opcode;
    result.words.at(2) = value;
  }
  result.words.at(jump - body_start) = indirect_call | (sample.reg << 8U);
  return result;
}

void publish_tuple(DspDebugState &expected, const Program &code, const Case sample) {
  switch (sample.producer) {
    case Producer::r: expected.cpu.r.at(sample.reg) = code.target; return;
    case Producer::n: expected.cpu.n.at(sample.reg) = 4; return;
    case Producer::m: expected.cpu.m.at(sample.reg) = word_mask; return;
    case Producer::none: throw Fault("No tuple producer exists in this case");
    default: throw Fault("Invalid tuple producer kind");
  }
}

void check_tuple_stage(const DspFetchCycleResult &cycle,
                       const DspMachine &machine,
                       DspDebugState &expected,
                       const Program &code,
                       const Case sample) {
  if (sample.producer == Producer::none) { return; }
  const auto &phase = pipeline::stages(cycle);
  if (cycle.completed_cycles == 6) {
    require(phase.ag2 && phase.ag2->address == body_start + 1,
            "Explicit tuple move has not yet reached its actual native E1 publication");
    control::check(machine, expected);
  }
  if (cycle.completed_cycles != 7) { return; }
  require(phase.execute1 && phase.execute1->address == body_start + 1 && phase.execute2 &&
              phase.execute2->address == body_start && phase.retirement_published,
          "Native tuple E1 publishes in the same selected period as older NOP E2");
  publish_tuple(expected, code, sample);
  auto after_nop = expected;
  ++after_nop.cpu.pc;
  ++after_nop.native_instructions;
  ++after_nop.native_cycles;
  control::check(machine, after_nop);
}

void publish_expected(DspDebugState &expected, const Program &code, const Case sample) {
  const auto pc = expected.cpu.pc;
  unsigned cost = 1;
  if (pc == code.jump) {
    ++expected.cpu.sp;
    ++expected.sc;
    expected.stack_high.at(expected.cpu.sp) = pc + 1;
    expected.stack_low.at(expected.cpu.sp) = expected.cpu.sr;
    expected.cpu.stack_pc = expected.stack_high.at(expected.cpu.sp);
    expected.cpu.stack_sr = expected.stack_low.at(expected.cpu.sp);
    expected.cpu.pc = code.target;
    cost = 3;
  } else if (pc == body_start + 1 && sample.producer != Producer::none) {
    expected.cpu.pc += 2;
    cost = 2;
    publish_tuple(expected, code, sample);
  } else {
    ++expected.cpu.pc;
  }
  ++expected.native_instructions;
  expected.native_cycles += cost;
}

struct Oracle {
  DspDebugState expected;
  unsigned retired = 0, calls = 0, held = 0;
};

unsigned holds(const Case sample) {
  return sample.producer != Producer::none && sample.gap < 2 ? 2 - sample.gap : 0U;
}

std::vector<unsigned> timeline(const Case sample) {
  if (sample.target == Target::self) { return {7, 8, 11, 14}; }
  const bool producer = sample.producer != Producer::none;
  std::vector<unsigned> result{7};
  if (producer) { result.push_back(8); }
  for (unsigned index = 0; index < sample.gap; ++index) {
    result.push_back((producer ? 10U : 8U) + index);
  }
  const unsigned call = 8 + (producer ? 2U : 0U) + sample.gap + holds(sample);
  result.push_back(call);
  result.push_back(call + 3);
  return result;
}

void step(DspFetchRun &run,
          DspMachine &machine,
          const Program &code,
          const Case sample,
          const std::vector<unsigned> &times,
          Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto clock = cycle.completed_cycles;
  check_tuple_stage(cycle, machine, oracle.expected, code, sample);
  const bool held = holds(sample) != 0 && clock >= 6 + sample.gap && clock < 8;
  require(phase.agu_dependencies == (held ? (1U << sample.reg) : 0U),
          "FM AGU windows permit exactly two/one/zero held JSR Decode periods");
  if (held) { ++oracle.held; }
  if (phase.execute1 && phase.execute1->address == code.jump) {
    require(!phase.e1_extension && phase.execution_captured,
            "Direct JSR consumes one actual native opcode at E1");
  }
  const bool due = oracle.retired < times.size() && clock == times.at(oracle.retired);
  require(phase.retirement_published == due,
          "Every native E2 follows the independent anchor/tuple/gap/call/target schedule");
  if (due) {
    if (!phase.execute2) { throw Fault("Native retirement lost its actual E2 receipt"); }
    require(phase.execute2->address == oracle.expected.cpu.pc,
            "Every native E2 retains the independently expected control path");
    if (phase.execute2->address == code.jump) { ++oracle.calls; }
    publish_expected(oracle.expected, code, sample);
    ++oracle.retired;
  }
  control::check(machine, oracle.expected);
  require(run.elapsed_ticks() == clock * pipeline::period,
          "Source readiness cannot change the native clock ledger");
}

void finish(DspFetchRun &run,
            DspMachine &machine,
            const Program &code,
            const Case sample,
            const std::vector<unsigned> &times,
            Oracle &oracle) {
  while (run.completed_cycles() < times.back()) { step(run, machine, code, sample, times, oracle); }
  require(oracle.retired == times.size() && oracle.held == holds(sample) &&
              oracle.calls == (sample.target == Target::self ? 3U : 1U),
          "All scheduled occurrences,stack pushes and explicit tuple holds are accounted for");
}

unsigned streams = 0, restores = 0, clones = 0;

void one_case(const Case sample, const bool snapshots = false) {
  const auto code = program(sample);
  const auto times = timeline(sample);
  DspMachine machine;
  Oracle oracle{.expected = control::prepare(machine, code.seeds, 2, code.words)};
  require(oracle.expected.cpu.sp == 0 && oracle.expected.sc == 0,
          "Actual native reset/data seeds start with an independently empty call stack");
  DspFetchRun run(control::contract(), machine);

  struct Cut {
    DspFetchCheckpoint checkpoint;
    Oracle oracle;
  };

  std::vector<Cut> cuts;
  while (run.completed_cycles() < times.back()) {
    if (snapshots) { cuts.push_back({.checkpoint = run.checkpoint(machine), .oracle = oracle}); }
    step(run, machine, code, sample, times, oracle);
  }
  for (const auto &cut : cuts) {
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
      run.restore(cut.checkpoint, machine);
      oracle = cut.oracle;
      finish(run, machine, code, sample, times, oracle);
      ++restores;
    }
    DspMachine target;
    DspFetchRun clone(cut.checkpoint, target);
    auto expected = cut.oracle;
    finish(clone, target, code, sample, times, expected);
    require(target.cpu().debug_state() == machine.cpu().debug_state() &&
                clone.elapsed_ticks() == run.elapsed_ticks(),
            "Fresh native ownership preserves every stack slot,tuple value and clock suffix");
    ++clones;
  }
  ++streams;
}
}

int main() {
  try {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const unsigned flags : {0U, frame_flags}) {
        for (const auto target : {Target::forward, Target::sequential, Target::self}) {
          one_case({.reg = reg, .target = target, .flags = flags});
        }
        for (const auto producer : {Producer::r, Producer::n, Producer::m}) {
          for (unsigned gap = 0; gap <= 4; ++gap) {
            one_case({.reg = reg, .producer = producer, .gap = gap, .flags = flags},
                     gap == 0 && flags == frame_flags);
          }
        }
      }
    }
    std::cout << "PASS JSR_PRODUCER_WINDOWS streams=" << streams << " restores=" << restores
              << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
