// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test::repeat_scalar {
inline constexpr unsigned entry = 0x100, prefix = entry + 1, body = prefix + 1,
                          following = body + 1;
inline constexpr unsigned data = 0x400, other_data = 0x200, canary = 0x654321, old_lc = 0x33;
inline constexpr unsigned immediate_count_limit = 0xfff;
inline constexpr unsigned immediate_rep = 0x0600a0, register_rep = 0x06c420;
inline constexpr unsigned absolute_jump = 0x0af080, byte_mask = 0xff, ordinary_omr = 0x300;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

struct Case {
  unsigned count = 3;
  bool reg = false;
  bool mutate = false;
  unsigned start = data;
  scalar_store::Case store{};
};

inline int stride(const Case sample) {
  switch (sample.store.mode) {
    case 0: return -3;
    case 1: return 3;
    case 2: return -1;
    case 3: return 1;
    case 4: return 0;
    default: throw Fault("Undefined own repeated scalar address mode");
  }
}

inline unsigned address(const Case sample, const unsigned index) {
  return static_cast<unsigned>(static_cast<int>(sample.start) +
                               (stride(sample) * static_cast<int>(index)));
}

inline std::pair<unsigned, unsigned> range(const Case sample) {
  const auto last = address(sample, sample.count - 1);
  return {std::min(last, sample.start) - 1, std::max(last, sample.start) + 1};
}

inline DspSpace space(const Case sample) {
  return sample.store.y ? DspSpace::y : DspSpace::x;
}

inline void prepare(DspMachine &machine, const Case sample) {
  require(sample.count > 0 && (sample.reg || sample.count <= immediate_count_limit),
          "Own REP fixture requires a positive encodable count");
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("lc", old_lc);
  program.immediate("x0", sample.count);
  program.immediate("r" + std::to_string(sample.store.reg), sample.start);
  program.immediate("n" + std::to_string(sample.store.reg), 3);
  const std::string accumulator = sample.store.b ? "b" : "a";
  for (unsigned part = 0; part < 3; ++part) {
    const auto value = (sample.store.value >> (part * word_bits)) & (part == 2 ? 0xffU : word_mask);
    program.immediate(accumulator + std::to_string(part), value);
  }
  program.immediate("sr", masked_status | sample.store.scale | sample.store.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned cursor = entry; cursor < entry + 32; ++cursor) {
    machine.bus().write({.address = cursor, .value = 0});
  }
  const auto opcode = sample.reg
                          ? register_rep
                          : immediate_rep | ((sample.count & byte_mask) << 8) | (sample.count >> 8);
  machine.bus().write({.address = prefix, .value = opcode});
  machine.bus().write({.address = body, .value = scalar_store::opcode(sample.store)});
  const auto [first, last] = range(sample);
  for (unsigned cursor = first; cursor <= last; ++cursor) {
    machine.bus().write({.space = space(sample), .address = cursor, .value = canary});
  }
  const auto opposite = sample.store.y ? DspSpace::x : DspSpace::y;
  for (const unsigned cursor : {other_data - 1, other_data, other_data + 1}) {
    machine.bus().write({.space = opposite, .address = cursor, .value = canary});
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
}

struct Trace {
  DspDebugState expected;
  unsigned bodies = 0, stores = 0, addresses = 0, decoded = 0, f1 = 0, f2 = 0;
  std::optional<std::uint64_t> prefix_decode = std::nullopt;
  std::optional<std::uint64_t> first_body_decode = std::nullopt;
  std::optional<std::uint64_t> last_body_e2 = std::nullopt;
};

inline void retirement(Trace &trace, const DspPipelineCycle &stages, const Case sample) {
  if (!stages.retirement_published) { return; }
  if (!stages.execute2) { throw Fault("Native REP retirement lost its actual instruction"); }
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
    trace.expected.cpu.sr |= scalar_store::expected(sample.store).flags;
    return;
  }
  require(word.address == entry || word.address == following,
          "Only anchor,REP,declared body and successor retire");
  trace.expected.cpu.pc = word.address + 1;
}

inline void
observe(Trace &trace, DspMachine &machine, const DspFetchCycleResult &cycle, const Case sample) {
  if (!cycle.pipeline) { throw Fault("Repeated scalar store lost its actual pipeline cycle"); }
  const auto &stages = *cycle.pipeline;
  if (stages.decode_accepted && stages.decode) {
    if (stages.decode->address == prefix) { trace.prefix_decode = cycle.completed_cycles; }
    if (stages.decode->address == body) {
      ++trace.decoded;
      if (!trace.first_body_decode) { trace.first_body_decode = cycle.completed_cycles; }
    }
  }
  if (stages.ag2_address) {
    require(stages.ag2_address->word.address == body,
            "Only the native repeated store may update an address register");
    ++trace.addresses;
    trace.expected.cpu.r.at(sample.store.reg) = address(sample, trace.addresses);
  }
  if (stages.memory) {
    const auto &memory = *stages.memory;
    const auto value = scalar_store::expected(sample.store).value;
    require(!memory.additional_data && memory.data.kind == DspNativeAccessKind::write &&
                memory.data.target.space == space(sample) && memory.data.value == value &&
                memory.data.target.address == address(sample, trace.stores) &&
                machine.bus().read(memory.data.target) == value,
            "Every native E1 scalar store has its independent address,value and actual RAM effect");
    ++trace.stores;
  }
  retirement(trace, stages, sample);
  if (stages.retirement_published && stages.execute2 && stages.execute2->address == body) {
    if (trace.last_body_e2) {
      require(cycle.completed_cycles == *trace.last_body_e2 + 1,
              "REP scalar bodies retain one-period nominal throughput");
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
      // Own-program latch experiment after real F2; no manufacturer bytes involved.
      constexpr unsigned illegal = 0x000007;
      machine.bus().write({.address = body, .value = illegal});
    }
  }
  observe(trace, machine, cycle, sample);
  if (machine.cpu().debug_state() != trace.expected) {
    std::cerr << "cycle=" << cycle.completed_cycles
              << " opcode=" << hex(scalar_store::opcode(sample.store), 6)
              << " count=" << sample.count << " bodies=" << trace.bodies
              << " stores=" << trace.stores << '\n';
  }
  require(machine.cpu().debug_state() == trace.expected,
          "Every REP period matches independent complete CPU,flags,LC,AGU and counter state");
  return cycle;
}

inline void memory(DspMachine &machine, const Case sample, const bool stored) {
  const auto [first, last] = range(sample);
  const auto step = stride(sample);
  const auto value = scalar_store::expected(sample.store).value;
  for (unsigned cursor = first; cursor <= last; ++cursor) {
    const int offset = static_cast<int>(cursor) - static_cast<int>(sample.start);
    const bool selected = step == 0 ? cursor == sample.start
                                    : offset % step == 0 && offset / step >= 0 &&
                                          static_cast<unsigned>(offset / step) < sample.count;
    require(machine.bus().read({.space = space(sample), .address = cursor}) ==
                (stored && selected ? value : canary),
            "REP stores preserve every gap and both adjacent canaries");
  }
  const auto opposite = sample.store.y ? DspSpace::x : DspSpace::y;
  for (const unsigned cursor : {other_data - 1, other_data, other_data + 1}) {
    require(machine.bus().read({.space = opposite, .address = cursor}) == canary,
            "REP scalar stores preserve the independent opposite-space sentinels");
  }
}

inline void completed(const Trace &trace, const Case sample, const DspFetchCycleResult &cycle) {
  require(trace.bodies == sample.count && trace.decoded == sample.count && trace.f1 == 1 &&
              trace.f2 == 1 && trace.stores == sample.count && trace.addresses == sample.count,
          "Every REP iteration retains one genuine body fetch and exact native writes/AGU updates");
  require(trace.prefix_decode && trace.first_body_decode && trace.last_body_e2 &&
              *trace.first_body_decode == *trace.prefix_decode + 5 &&
              cycle.completed_cycles == *trace.last_body_e2 + 1,
          "Native REP setup,body and successor retain independent nominal timing");
}

struct Saved {
  DspFetchCheckpoint checkpoint;
  Trace trace;
};

inline void finish(DspFetchRun &run, DspMachine &machine, Trace &trace, const Case sample) {
  for (unsigned index = 0; index < sample.count + 64; ++index) {
    const auto cycle = advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != following + 1) { continue; }
    completed(trace, sample, cycle);
    memory(machine, sample, true);
    return;
  }
  throw Fault("Repeated scalar store exceeded its count-derived bound");
}

inline unsigned one(const Case sample, const bool checkpoints = false) {
  DspMachine machine;
  prepare(machine, sample);
  Trace trace{.expected = machine.cpu().debug_state()};
  DspFetchRun run(scalar_store::contract(), machine);
  std::vector<Saved> saved;
  for (unsigned index = 0; index < sample.count + 64; ++index) {
    if (checkpoints) { saved.push_back({.checkpoint = run.checkpoint(machine), .trace = trace}); }
    const auto cycle = advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != following + 1) { continue; }
    completed(trace, sample, cycle);
    memory(machine, sample, true);
    const auto expected = machine.cpu().debug_state();
    for (const auto &cut : saved) {
      for (unsigned replay = 0; replay < 2; ++replay) {
        run.restore(cut.checkpoint, machine);
        auto tail = cut.trace;
        finish(run, machine, tail, sample);
        require(machine.cpu().debug_state() == expected,
                "Every restored REP suffix matches the independent literal final state");
      }
    }
    return static_cast<unsigned>(saved.size());
  }
  throw Fault("Repeated scalar store exceeded its count-derived bound");
}

inline bool body_refusal(const std::string &error) {
  return error.contains("Unqualified REP body read P:") ||
         error.contains("Unqualified repeated Decode read P:");
}

inline void refused(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  Trace trace{.expected = machine.cpu().debug_state()};
  DspFetchRun run(scalar_store::contract(), machine);
  const auto error = checkpoint::rejects(
      [&] {
        for (unsigned index = 0; index < sample.count + 64; ++index) {
          static_cast<void>(advance(run, machine, trace, sample));
        }
      },
      "Expected current repeated scalar-store refusal");
  require(body_refusal(error) && error.contains("opcode=") && error.contains("pc=") &&
              error.contains("omr=") && trace.bodies == 0 && trace.stores == 0 &&
              machine.cpu().state().r.at(sample.store.reg) == sample.start,
          "Current REP scalar refusal precedes actual body/address effects");
  memory(machine, sample, false);
  std::cout << "Baseline REP refusal: " << error << '\n';
}
}
