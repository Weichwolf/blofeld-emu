// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_transfer_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld::test::transfer_pipeline {
namespace {
enum class Producer : std::uint8_t { registers, short_literal, long_literal, memory_x, memory_y };

struct Case {
  char kind = 'r';
  unsigned index = 0;
  unsigned gap = 0;
  unsigned upper = 0;
  Producer producer = Producer::registers;
};

unsigned expected_pointer(const char kind) {
  if (kind == 'r') { return 4; }
  if (kind == 'n') { return 0; }
  if (kind == 'm') { return 10; }
  throw Fault("Invalid own AGU destination kind");
}

std::string producer(const Case sample) {
  const auto reg = std::string(1, sample.kind) + std::to_string(sample.index);
  const auto value = sample.kind == 'm' ? ((sample.upper << 16U) | 15U) : 1U;
  switch (sample.producer) {
    case Producer::registers: return "move x1," + reg;
    case Producer::short_literal: return "move #<" + number(value) + ',' + reg;
    case Producer::long_literal: return "move #>" + number(value) + ',' + reg;
    case Producer::memory_x: return "move x:(r" + std::to_string(sample.index) + ")," + reg;
    case Producer::memory_y: return "move y:(r" + std::to_string(sample.index) + ")," + reg;
    default: throw Fault("Invalid own AGU producer form");
  }
}

void one_dependency(const Case sample) {
  const auto reg = std::to_string(sample.index);
  const auto value = sample.kind == 'm' ? ((sample.upper << 16U) | 15U) : 1U;
  const auto words = sample.producer == Producer::long_literal ? 2U : 1U;
  const auto ea = sample.kind == 'r' ? 1U : 7U;
  const auto pointer = expected_pointer(sample.kind);
  const auto loaded = sample.producer == Producer::memory_x && sample.kind != 'r' ? value : other;
  // Primary A.2.4 and independently observed NXP producer-form windows.
  constexpr std::array ordinary_delays{3U, 2U, 1U, 0U};
  constexpr std::array literal_delays{2U, 1U, 0U, 0U};
  const auto stalls = words == 2 ? literal_delays.at(sample.gap) : ordinary_delays.at(sample.gap);
  const auto cycles = 7 + words + sample.gap + stalls;
  std::vector<std::string> code{producer(sample)};
  code.insert(code.end(), sample.gap, "nop");
  code.push_back("move x:(r" + reg + ")+n" + reg + ",y1");
  DspMachine machine;
  prepare(machine, code, {.index = sample.index, .source = value});
  machine.bus().write({.space = DspSpace::x, .address = ea, .value = loaded});
  if (sample.producer == Producer::memory_x || sample.producer == Producer::memory_y) {
    const auto space = sample.producer == Producer::memory_x ? DspSpace::x : DspSpace::y;
    machine.bus().write({.space = space, .address = 7, .value = value});
  }
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  unsigned observed_stalls = 0;
  bool consumer_address = false;
  for (unsigned clock = 0; clock < cycles; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = stages(cycle);
    if (phase.agu_dependencies != 0) {
      ++observed_stalls;
      require(phase.agu_dependencies == (1U << sample.index) && !phase.decode_accepted &&
                  phase.advanced && !cycle.issued_address && !cycle.bus.fetch,
              "A real decoded register-set conflict holds Decode/F2/F1 while older stages drain");
    }
    if (phase.ag1_address && phase.ag1_address->word.address == body + words + sample.gap) {
      consumer_address = true;
      require(phase.ag1_address->update.address == ea && phase.ag1_address->update.value == pointer,
              "AG1 uses the now-ready actual R/N/M and native modifier caches, not stale inputs");
    }
  }
  auto expected = before;
  expected.cpu.r.at(sample.index) = pointer;
  if (sample.kind == 'n') { expected.cpu.n.at(sample.index) = value; }
  if (sample.kind == 'm') { expected.cpu.m.at(sample.index) = value; }
  expected.cpu.y = (std::uint64_t{loaded} << word_bits) + memory_pipeline::half;
  expected.cpu.pc += words + sample.gap + 1;
  expected.native_instructions += sample.gap + 2;
  expected.native_cycles += words + sample.gap + 1;
  require(observed_stalls == stalls && consumer_address &&
              machine.cpu().debug_state() == expected && run.completed_cycles() == cycles &&
              run.elapsed_ticks() == cycles * pipeline::period &&
              run.timer_clk2_edges() == cycles / 2,
          "Literal AGU dependency clocks and complete final CPU state differ for " +
              producer(sample) + " gap=" + std::to_string(sample.gap));
}

void loaded_pointer_replaces_agu() {
  DspMachine machine;
  const std::array<std::string, 2> code{"move x:(r0)+,r0", "move x:(r0)+,x0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = 1});
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = other});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  require(machine.cpu().state().r[0] == 0,
          "Actual AG2 pointer update precedes its same-R data load");
  clocks(run, machine, 1);
  require(machine.cpu().state().r[0] == 1,
          "Actual loaded R replaces its own earlier AG2 value in the retention overlay at E1");
  clocks(run, machine, 1);
  require(machine.cpu().state().r[0] == 1, "Older E2 cannot resurrect the pre-load AG2 value");
  clocks(run, machine, 4);
  require(machine.cpu().state().pc == body + 2 && machine.cpu().state().r[0] == 2 &&
              machine.cpu().state().x == pipeline::unit + other && run.completed_cycles() == 11,
          "The next native AGU uses the actual loaded pointer after three real decode holds");
}

void ordinary_updates_do_not_stall() {
  DspMachine machine;
  const std::array<std::string, 4> code{
      "move x:(r0)+,x0", "move x:(r0)+,x0", "move x:(r0)+,x0", "move x:(r0)+,x0"};
  prepare(machine, code);
  for (const auto address : {7U, 0U, 1U, 2U}) {
    machine.bus().write({.space = DspSpace::x, .address = address, .value = other});
  }
  DspFetchRun run(contract(), machine);
  for (unsigned clock = 0; clock < 10; ++clock) {
    require(stages(run.advance_cycle(machine, fetch_clock::input())).agu_dependencies == 0,
            "Implicit native AG2 post-updates never masquerade as explicit R destinations");
  }
  require(machine.cpu().state().pc == body + 4 && machine.cpu().state().r[0] == 3 &&
              machine.cpu().state().x == pipeline::unit + other,
          "A filled ordinary memory stream retains one retirement per reference clock");
}

void unrelated_group(const char kind) {
  DspMachine machine;
  const std::array code{"move x1," + std::string(1, kind) + "4", std::string("move x:(r0)+n0,y1")};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = other});
  DspFetchRun run(contract(), machine);
  for (unsigned clock = 0; clock < 8; ++clock) {
    require(stages(run.advance_cycle(machine, fetch_clock::input())).agu_dependencies == 0,
            "A different register set does not invent a decoded address dependency");
  }
  require(machine.cpu().state().r[0] == 2 && machine.cpu().state().pc == body + 2 &&
              machine.cpu().state().y ==
                  (std::uint64_t{other} << word_bits) + memory_pipeline::half,
          "An unrelated producer keeps nominal throughput and actual memory operands");
}
}

void dependency_checks() {
  for (const auto form : {Producer::registers,
                          Producer::short_literal,
                          Producer::long_literal,
                          Producer::memory_x,
                          Producer::memory_y}) {
    for (const char kind : {'r', 'n', 'm'}) {
      for (unsigned index = 0; index < 8; ++index) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          one_dependency({.kind = kind, .index = index, .gap = gap, .producer = form});
        }
      }
    }
  }
  constexpr std::array additional_upper_bytes{1U, 0x7fU, 0x80U, 0xffU};
  for (const auto form :
       {Producer::registers, Producer::long_literal, Producer::memory_x, Producer::memory_y}) {
    for (const auto upper : additional_upper_bytes) {
      for (unsigned index = 0; index < 8; ++index) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          one_dependency(
              {.kind = 'm', .index = index, .gap = gap, .upper = upper, .producer = form});
        }
      }
    }
  }
  loaded_pointer_replaces_agu();
  ordinary_updates_do_not_stall();
  for (const char kind : {'r', 'n', 'm'}) { unrelated_group(kind); }
}
}
