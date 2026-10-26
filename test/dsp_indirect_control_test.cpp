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
                 .seeds = {control::sr_move, masked_status, move_r | (sample.reg << 16U), old_r},
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
  result.words.at(jump - body_start) = control::indirect_jump | (sample.reg << 8U);
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

void one_case(const Case sample) {
  const auto code = program(sample);
  DspMachine machine;
  auto expected = control::prepare(machine, code.seeds, 2, code.words);
  DspFetchRun run(control::contract(), machine);
  const bool producer = sample.producer != Producer::none;
  const bool self = sample.target == Target::self;
  const unsigned extra = producer && sample.gap < 2 ? 2 - sample.gap : 0U;
  const unsigned nominal = producer ? 2 + sample.gap : sample.gap;
  const unsigned last = control::first_retirement + (self ? 7U : nominal + extra + 4);
  unsigned retired = 0;
  unsigned jumps = 0;
  unsigned held = 0;
  while (run.completed_cycles() < last) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    check_tuple_stage(cycle, machine, expected, code, sample);
    if (phase.agu_dependencies != 0) {
      require(phase.agu_dependencies == (1U << sample.reg),
              "Only the actual producer's R/N/M tuple holds direct JMP Decode");
      ++held;
    }
    if (phase.execute1 && phase.execute1->address == code.jump) {
      require(!phase.e1_extension && phase.execution_captured,
              "JMP(Rn) uses one actual opcode and a native E1 result");
    }
    if (!phase.retirement_published) { continue; }
    if (!phase.execute2) { throw Fault("Retirement lost its actual E2 word"); }
    require(phase.execute2 && phase.execute2->address == expected.cpu.pc,
            "Native indirect control preserves actual retirement order");
    if (phase.execute2->address == code.jump) { ++jumps; }
    publish_expected(expected, code, sample);
    ++retired;
    control::check(machine, expected);
  }
  require(retired == (self ? 4U : 3U + sample.gap + (producer ? 1U : 0U)) &&
              jumps == (self ? 3U : 1U) && held == extra,
          "Independent three-period JMP/target window and tuple hazards match every occurrence");
}

void restored_jump() {
  const Case sample{.reg = 1};
  const auto code = program(sample);
  for (unsigned cut = 4; cut <= 10; ++cut) {
    DspMachine machine;
    auto expected = control::prepare(machine, code.seeds, 2, code.words);
    expected.cpu.pc = code.target + 1;
    expected.native_instructions += 3;
    expected.native_cycles += 5;
    DspFetchRun run(control::contract(), machine);
    pipeline::clocks(run, machine, cut);
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      pipeline::clocks(run, machine, 11 - cut);
      control::check(machine, expected);
    }
  }
}
}

int main() {
  try {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const auto target : {Target::forward, Target::sequential, Target::self}) {
        one_case({.reg = reg, .target = target});
      }
      for (const auto producer : {Producer::r, Producer::n, Producer::m}) {
        for (unsigned gap = 0; gap <= 4; ++gap) {
          one_case({.reg = reg, .producer = producer, .gap = gap});
        }
      }
    }
    restored_jump();
    std::cout << "144 native indirect control/tuple hazard cases and14 restored suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
