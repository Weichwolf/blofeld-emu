// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::repeat_clock {
inline constexpr unsigned entry = 0x100, prefix = entry + 1, body = prefix + 1,
                          following = body + 1;
inline constexpr unsigned data = 0x400, high = 0x234567, low = 0x12345, old_lc = 0x33,
                          canary = 0x654321;
inline constexpr unsigned immediate_rep = 0x0600a0, register_rep = 0x06c420, long_store = 0x485c00;
inline constexpr unsigned absolute_jump = 0x0af080, byte_mask = 0xff, ordinary_omr = 0x300;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

struct Case {
  unsigned count;
  bool memory = false, reg = false;
  bool mutate = false, checkpoints = false;
};

inline void prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("sr", masked_status);
  program.immediate("lc", old_lc);
  program.immediate("x0", sample.count);
  program.immediate("r4", data);
  program.immediate("a0", low);
  program.immediate("a1", high);
  program.immediate("a2", 0);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned address = entry; address < entry + 32; ++address) {
    machine.bus().write({.address = address, .value = 0});
  }
  const auto opcode = sample.reg
                          ? register_rep
                          : immediate_rep | ((sample.count & byte_mask) << 8) | (sample.count >> 8);
  machine.bus().write({.address = prefix, .value = opcode});
  machine.bus().write({.address = body, .value = sample.memory ? long_store : 0});
  for (unsigned index = 0; index < (sample.memory ? sample.count : 0) + 2; ++index) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      machine.bus().write({.space = space, .address = data + index, .value = canary});
    }
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
}

struct Trace {
  DspDebugState expected;
  unsigned bodies = 0, stores = 0, decoded = 0, f1 = 0, f2 = 0;
  std::optional<std::uint64_t> prefix_decode = std::nullopt;
  std::optional<std::uint64_t> first_body_decode = std::nullopt;
  std::optional<std::uint64_t> last_body_e2 = std::nullopt;
};

inline void observe_retirement(Trace &trace, const DspPipelineCycle &stages, const Case sample) {
  if (!stages.retirement_published) { return; }
  if (!stages.execute2) { throw Fault("Retired REP test stage lost its actual word"); }
  const auto word = *stages.execute2;
  ++trace.expected.native_instructions;
  if (word.address == prefix) {
    trace.expected.cpu.pc = body;
    trace.expected.cpu.lc = sample.count;
    trace.expected.native_cycles += 5;
    return;
  }
  ++trace.expected.native_cycles;
  if (word.address == body) {
    ++trace.bodies;
    const auto remaining = sample.count - trace.bodies;
    trace.expected.cpu.pc = remaining == 0 ? following : body;
    trace.expected.cpu.lc = remaining == 0 ? old_lc : remaining;
    if (sample.memory) { trace.expected.cpu.sr |= sticky; }
    return;
  }
  require(word.address == entry || word.address == following,
          "Only the declared anchor,REP,body and successor may retire");
  trace.expected.cpu.pc = word.address + 1;
}

inline void observe(Trace &trace, const DspFetchCycleResult &cycle, const Case sample) {
  if (!cycle.pipeline) { throw Fault("Missing real automatic REP pipeline cycle"); }
  const auto &stages = *cycle.pipeline;
  if (stages.decode_accepted && stages.decode) {
    if (stages.decode->address == prefix) { trace.prefix_decode = cycle.completed_cycles; }
    if (stages.decode->address == body) {
      ++trace.decoded;
      if (!trace.first_body_decode) { trace.first_body_decode = cycle.completed_cycles; }
    }
  }
  if (stages.ag2_address) {
    require(sample.memory && stages.ag2_address->word.address == body,
            "Only a genuine repeated memory instruction may publish its AG2 result");
    ++trace.expected.cpu.r[4];
  }
  if (stages.memory) {
    const auto &memory = *stages.memory;
    require(sample.memory && memory.additional_data && memory.data.value == high &&
                memory.additional_data->value == low &&
                memory.data.target.address == data + trace.stores &&
                memory.additional_data->target.address == data + trace.stores,
            "Each actual X/Y long store must match independent literal data/address expectations");
    ++trace.stores;
  }
  observe_retirement(trace, stages, sample);
  if (stages.retirement_published && stages.execute2 && stages.execute2->address == body) {
    if (trace.last_body_e2) {
      require(cycle.completed_cycles == *trace.last_body_e2 + 1,
              "Consecutive native REP bodies must retain their nominal single-period throughput");
    }
    trace.last_body_e2 = cycle.completed_cycles;
  }
}

inline DspFetchCycleResult
advance(DspFetchRun &run, DspMachine &machine, Trace &trace, const Case sample) {
  const auto before_fetch = machine.reference_fetch();
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  if (cycle.issued_address == body) { ++trace.f1; }
  if (cycle.bus.fetch && before_fetch.pending_address == body) {
    ++trace.f2;
    if (sample.mutate) {
      // Own-program coherence experiment, after the genuine F2 capture.
      machine.bus().write({.address = body, .value = 0x000007});
    }
  }
  observe(trace, cycle, sample);
  require(machine.cpu().debug_state() == trace.expected,
          "Every original/restored clock must match independent full native state expectations");
  return cycle;
}

inline void completed(const Trace &trace, const Case sample, const DspFetchCycleResult &cycle) {
  require(trace.bodies == sample.count && trace.decoded == sample.count && trace.f1 == 1 &&
              trace.f2 == 1 && trace.stores == (sample.memory ? sample.count : 0),
          "All actual iterations must retain one genuine body fetch and exact native writes");
  require(trace.prefix_decode && trace.first_body_decode && trace.last_body_e2 &&
              *trace.first_body_decode == *trace.prefix_decode + 5 &&
              cycle.completed_cycles == *trace.last_body_e2 + 1,
          "Actual REP setup and successor must retain independent nominal timing");
}

struct Saved {
  DspFetchCheckpoint checkpoint;
  Trace trace;
};

inline void suffix(DspFetchRun &run,
                   DspMachine &machine,
                   const Saved &saved,
                   const Case sample,
                   const DspDebugState &expected) {
  run.restore(saved.checkpoint, machine);
  auto trace = saved.trace;
  require(machine.cpu().debug_state() == trace.expected,
          "Restored joint clock/native/address/F2 state must retain its independent CPU witness");
  for (unsigned index = 0; index < sample.count + 64; ++index) {
    const auto cycle = advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != following + 1) { continue; }
    completed(trace, sample, cycle);
    require(machine.cpu().debug_state() == expected,
            "Every period suffix must retain full native counters and all architectural state");
    return;
  }
  throw Fault("Restored automatic REP exceeded its explicit clock bound");
}

inline unsigned one(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  Trace trace{.expected = machine.cpu().debug_state()};
  auto contract = fetch_clock::contract();
  contract.execution = execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  std::vector<Saved> snapshots;
  for (unsigned index = 0; index < sample.count + 64; ++index) {
    if (sample.checkpoints) {
      snapshots.push_back({.checkpoint = run.checkpoint(machine), .trace = trace});
    }
    const auto cycle = advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != following + 1) { continue; }
    completed(trace, sample, cycle);
    const auto expected = machine.cpu().debug_state();
    for (const auto &saved : snapshots) { suffix(run, machine, saved, sample, expected); }
    return static_cast<unsigned>(snapshots.size());
  }
  throw Fault("Own automatic REP exceeded its explicit count-derived clock bound");
}
}
