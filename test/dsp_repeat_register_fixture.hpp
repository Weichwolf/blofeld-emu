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

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test::repeat_register {
inline constexpr unsigned entry = 0x100, prefix = entry + 1, body = prefix + 1,
                          following = body + 1;
inline constexpr unsigned data = 0x400, canary = 0x654321, old_lc = 0x33;
inline constexpr unsigned linear = 0xffffff, offset = 3, immediate_limit = 0xfff;
inline constexpr unsigned bound_slack = 96, illegal_opcode = 0x000007;
inline constexpr unsigned immediate_rep = 0x0600a0, x0_rep = 0x06c420, a1_rep = 0x06cc20;
inline constexpr unsigned jump = 0x0af080, ordinary_omr = 0x300, byte_mask = 0xff;
inline constexpr unsigned data_first = 4, data_last = 7, offset_first = 24, offset_last = 31;
inline constexpr std::array data_values{0x112233U, 0x445566U, 0x778899U, 0xaabbccU};
inline constexpr std::array data_names{"x0", "x1", "y0", "y1"};
inline constexpr unsigned a0_seed = 0xd4e5f6, a1_seed = 0x123456, a2_seed = 0xe9;
inline constexpr unsigned b0_seed = 0x6c7d8e, b1_seed = 0x3f4a5b, b2_seed = 0x12;
inline constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

enum class Counter : std::uint8_t { immediate, x0, a1 };

struct Case {
  unsigned source = data_first, mode = 3, reg = 3, count = 3, flags = 1;
  Counter counter = Counter::a1;
  bool y = false, mutate = false;
  std::optional<unsigned> count_input = std::nullopt;
};

inline unsigned count_value(const Case sample) {
  return sample.count_input.value_or(sample.count);
}

inline bool offset_source(const Case sample) {
  return sample.source >= offset_first && sample.source <= offset_last;
}

inline unsigned value(const Case sample) {
  if (sample.source == data_first && sample.counter == Counter::x0) { return count_value(sample); }
  if (sample.source >= data_first && sample.source <= data_last) {
    return data_values.at(sample.source - data_first);
  }
  if (!offset_source(sample)) { throw Fault("Undefined own raw store source field"); }
  const bool used_offset = sample.mode == 0 || sample.mode == 1 || sample.mode == 5;
  if (sample.source - offset_first == sample.reg && used_offset) { return offset; }
  constexpr unsigned offset_seed = 0xab1234;
  constexpr unsigned offset_stride = 0x123;
  return offset_seed + ((sample.source - offset_first) * offset_stride);
}

inline unsigned opcode(const Case sample) {
  constexpr unsigned base = 0x404000;
  constexpr unsigned field_high_mask = 0x18;
  constexpr unsigned field_low_mask = 7;
  constexpr unsigned high_shift = 17;
  constexpr unsigned low_shift = 16;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned reg_shift = 8;
  constexpr unsigned y_bit = 0x80000;
  static_cast<void>(value(sample));
  return base | ((sample.source & field_high_mask) << high_shift) |
         ((sample.source & field_low_mask) << low_shift) | (sample.mode << mode_shift) |
         (sample.reg << reg_shift) | (sample.y ? y_bit : 0U);
}

inline unsigned prefix_opcode(const Case sample) {
  switch (sample.counter) {
    case Counter::immediate:
      return immediate_rep | ((count_value(sample) & byte_mask) << 8U) |
             (count_value(sample) >> 8U);
    case Counter::x0: return x0_rep;
    case Counter::a1: return a1_rep;
    default: throw Fault("Undefined own REP count form");
  }
}

inline int stride(const Case sample) {
  switch (sample.mode) {
    case 0: return -static_cast<int>(offset);
    case 1: return static_cast<int>(offset);
    case 2: return -1;
    case 3: return 1;
    case 4:
    case 5: return 0;
    case 7: return -1;
    default: throw Fault("Undefined own REP raw-store address mode");
  }
}

inline unsigned address(const Case sample, const unsigned index) {
  int first = static_cast<int>(data);
  if (sample.mode == 5) { first += static_cast<int>(offset); }
  if (sample.mode == 7) { --first; }
  return static_cast<unsigned>(first + (stride(sample) * static_cast<int>(index)));
}

inline unsigned pointer(const Case sample, const unsigned completed) {
  return static_cast<unsigned>(static_cast<int>(data) +
                               (stride(sample) * static_cast<int>(completed)));
}

inline unsigned body_clocks(const Case sample) {
  return sample.mode == 5 || sample.mode == 7 ? 2U : 1U;
}

inline DspSpace space(const Case sample) {
  return sample.y ? DspSpace::y : DspSpace::x;
}

inline std::pair<unsigned, unsigned> range(const Case sample) {
  const auto first = address(sample, 0);
  const auto last = address(sample, sample.count - 1);
  return {std::min(first, last) - 1, std::max(first, last) + 1};
}

inline void validate_seeds(const DspState &state, const Case sample) {
  constexpr std::uint64_t a_low = a0_seed;
  constexpr std::uint64_t a_high = 0xe9000000000000;
  constexpr std::uint64_t b_value = 0x123f4a5b6c7d8e;
  constexpr unsigned a_default_middle = a1_seed;
  const auto x0 = sample.counter == Counter::x0 ? count_value(sample) : data_values[0];
  const auto a1 = sample.counter == Counter::a1 ? count_value(sample) : a_default_middle;
  require(state.x == (static_cast<std::uint64_t>(data_values[1]) << word_bits | x0) &&
              state.y ==
                  (static_cast<std::uint64_t>(data_values[3]) << word_bits | data_values[2]) &&
              state.a == (a_high | (static_cast<std::uint64_t>(a1) << word_bits) | a_low) &&
              state.b == b_value,
          "Declared native register seeds match independent raw24 and signed56 field expectations");
}

inline void cells(DspMachine &machine, const Case sample, const unsigned stores) {
  const auto [first, last] = range(sample);
  const auto step = stride(sample);
  const auto first_address = address(sample, 0);
  for (unsigned cursor = first; cursor <= last; ++cursor) {
    const int distance = static_cast<int>(cursor) - static_cast<int>(first_address);
    const bool selected = step == 0 ? cursor == first_address
                                    : distance % step == 0 && distance / step >= 0 &&
                                          static_cast<unsigned>(distance / step) < stores;
    require(machine.bus().read({.space = space(sample), .address = cursor}) ==
                (stores != 0 && selected ? value(sample) : canary),
            "Raw REP stores preserve every independent gap and neighboring canary");
  }
  const auto opposite = sample.y ? DspSpace::x : DspSpace::y;
  for (unsigned cursor = first; cursor <= last; ++cursor) {
    require(machine.bus().read({.space = opposite, .address = cursor}) == canary,
            "Raw REP stores preserve all opposite-space canaries");
  }
}

inline void prepare(DspMachine &machine, const Case sample) {
  require(sample.count > 0 &&
              (sample.counter != Counter::immediate || count_value(sample) <= immediate_limit),
          "Own raw REP case requires a positive encodable count");
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("lc", old_lc);
  for (unsigned index = 0; index < data_values.size(); ++index) {
    program.immediate(data_names.at(index),
                      index == 0 && sample.counter == Counter::x0 ? count_value(sample)
                                                                  : data_values.at(index));
  }
  program.immediate("a0", a0_seed);
  program.immediate("a1", sample.counter == Counter::a1 ? count_value(sample) : a1_seed);
  program.immediate("a2", a2_seed);
  program.immediate("b0", b0_seed);
  program.immediate("b1", b1_seed);
  program.immediate("b2", b2_seed);
  const auto reg = std::to_string(sample.reg);
  program.immediate("r" + reg, data);
  program.immediate("n" + reg, offset);
  program.immediate("m" + reg, linear);
  if (offset_source(sample)) {
    program.immediate("n" + std::to_string(sample.source - offset_first), value(sample));
  }
  program.immediate("sr", masked_status | sample.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned cursor = entry; cursor < entry + 32; ++cursor) {
    machine.bus().write({.address = cursor});
  }
  machine.bus().write({.address = prefix, .value = prefix_opcode(sample)});
  machine.bus().write({.address = body, .value = opcode(sample)});
  const auto [first, last] = range(sample);
  for (unsigned cursor = first; cursor <= last; ++cursor) {
    machine.bus().write({.space = space(sample), .address = cursor, .value = canary});
  }
  const auto opposite = sample.y ? DspSpace::x : DspSpace::y;
  for (unsigned cursor = first; cursor <= last; ++cursor) {
    machine.bus().write({.space = opposite, .address = cursor, .value = canary});
  }
  const auto state = machine.cpu().state();
  validate_seeds(state, sample);
  require(state.pc == entry && state.r.at(sample.reg) == data && state.m.at(sample.reg) == linear &&
              state.sr == (masked_status | sample.flags),
          "Actual native setup supplies the declared independent raw REP context");
  if (offset_source(sample)) {
    require(state.n.at(sample.source - offset_first) == value(sample),
            "Native setup retains exact N source bits");
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
}

// Fixed selected-reference stage contract: cold Decode starts at3, REP at4,
// setup takes5 periods. Each indexed/predecrement body adds one E1 hold.
// These expectations do not assert measured silicon sample edges.
struct Times {
  unsigned decode, ag1, ag2, e1, e2;
};

inline unsigned delayed(const unsigned nominal, const Case sample, const bool include_hold) {
  constexpr unsigned first_hold = 12;
  if (body_clocks(sample) == 1 || nominal < first_hold) { return nominal; }
  const auto holds = nominal - first_hold + (include_hold ? 1U : 0U);
  return nominal + std::min(sample.count, holds);
}

inline Times times(const unsigned index, const Case sample) {
  return {.decode = delayed(9 + index, sample, true),
          .ag1 = delayed(10 + index, sample, true),
          .ag2 = delayed(11 + index, sample, true),
          .e1 = delayed(12 + index, sample, true),
          .e2 = delayed(13 + index, sample, false)};
}

struct Trace {
  DspDebugState expected;
  DspDebugState initial;
  unsigned bodies = 0, stores = 0, addresses = 0, decoded = 0, f1 = 0, f2 = 0;
  std::optional<std::uint64_t> prefix_decode = std::nullopt;
  std::optional<std::uint64_t> first_body_decode = std::nullopt;
  std::optional<std::uint64_t> last_body_e2 = std::nullopt;
};

inline DspDebugState expected_state(const Trace &trace, const Case sample, const unsigned clock) {
  auto expected = trace.initial;
  if (clock >= 7) {
    expected.cpu.pc = prefix;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  if (clock >= 8) {
    expected.cpu.pc = body;
    expected.cpu.lc = sample.count;
    ++expected.native_instructions;
    expected.native_cycles += 5;
  }
  for (unsigned index = 0; index < sample.count; ++index) {
    const auto schedule = times(index, sample);
    if (clock >= schedule.ag2) { expected.cpu.r.at(sample.reg) = pointer(sample, index + 1); }
    if (clock < schedule.e2) { continue; }
    ++expected.native_instructions;
    expected.native_cycles += body_clocks(sample);
    const auto remaining = sample.count - index - 1;
    expected.cpu.pc = remaining == 0 ? following : body;
    expected.cpu.lc = remaining == 0 ? old_lc : remaining;
  }
  if (clock >= delayed(13 + sample.count, sample, false)) {
    expected.cpu.pc = following + 1;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  return expected;
}

inline void program_phases(Trace &trace,
                           const DspPipelineCycle &phase,
                           const Case sample,
                           const unsigned clock) {
  const bool prefix_decode =
      phase.decode_accepted && phase.decode && phase.decode->address == prefix;
  const bool body_decode = phase.decode_accepted && phase.decode && phase.decode->address == body;
  bool body_due = false;
  bool hold_due = false;
  std::optional<unsigned> retired = std::nullopt;
  if (clock == 7) { retired = entry; }
  if (clock == 8) { retired = prefix; }
  if (clock == delayed(13 + sample.count, sample, false)) { retired = following; }
  for (unsigned index = 0; index < sample.count; ++index) {
    const auto schedule = times(index, sample);
    body_due = body_due || clock == schedule.decode;
    hold_due = hold_due || (body_clocks(sample) == 2 && clock + 1 == schedule.e1);
    if (clock == schedule.e2) { retired = body; }
  }
  require(prefix_decode == (clock == 4) && body_decode == body_due &&
              phase.retirement_published == retired.has_value() && !phase.memory_source_stall &&
              phase.advanced != hold_due,
          "Fixed Decode, E1-hold and E2 positions are independent of observed phase progress");
  if (retired) {
    require(phase.execute2 && phase.execute2->address == *retired,
            "Every scheduled E2 belongs to its independently expected instruction");
    if (*retired == body) {
      ++trace.bodies;
      trace.last_body_e2 = clock;
    }
  }
  if (prefix_decode) { trace.prefix_decode = clock; }
  if (body_due) {
    ++trace.decoded;
    if (!trace.first_body_decode) { trace.first_body_decode = clock; }
  }
}

inline void memory_phases(Trace &trace,
                          const DspPipelineCycle &phase,
                          const Case sample,
                          const unsigned clock) {
  std::optional<unsigned> ag1 = std::nullopt;
  std::optional<unsigned> ag2 = std::nullopt;
  std::optional<unsigned> e1 = std::nullopt;
  for (unsigned index = 0; index < sample.count; ++index) {
    const auto schedule = times(index, sample);
    if (clock == schedule.ag1) { ag1 = index; }
    if (clock == schedule.ag2) { ag2 = index; }
    if (clock == schedule.e1) { e1 = index; }
  }
  require(phase.ag1_address.has_value() == ag1.has_value() &&
              phase.ag2_address.has_value() == ag2.has_value() &&
              phase.memory.has_value() == e1.has_value(),
          "Native address and data effects occur at independently scheduled AG1/AG2/E1 positions");
  if (ag1 && phase.ag1_address) {
    const auto &stage = *phase.ag1_address;
    const bool changes_pointer = sample.mode != 4 && sample.mode != 5;
    require(
        stage.word.address == body && stage.word.value == opcode(sample) && !stage.published &&
            stage.pointer_update == changes_pointer && stage.update.index == sample.reg &&
            stage.update.before == pointer(sample, *ag1) &&
            stage.update.value == pointer(sample, *ag1 + 1) &&
            stage.update.address == address(sample, *ag1),
        "Every scheduled AG1 contains the independently calculated pointer and effective address");
  }
  if (ag2 && phase.ag2_address) {
    const auto &stage = *phase.ag2_address;
    require(stage.word.address == body && stage.published &&
                stage.update.address == address(sample, *ag2),
            "Every scheduled AG2 publishes its own native address");
    ++trace.addresses;
  }
  if (e1 && phase.memory) {
    const auto &memory = *phase.memory;
    require(!memory.additional_data && memory.data.kind == DspNativeAccessKind::write &&
                memory.data.target.space == space(sample) && memory.data.value == value(sample) &&
                memory.data.target.address == address(sample, *e1),
            "Every scheduled E1 stores the independently expected raw bits at its exact address");
    ++trace.stores;
  }
}

inline void
observe(Trace &trace, DspMachine &machine, const DspFetchCycleResult &cycle, const Case sample) {
  if (!cycle.pipeline) { throw Fault("Raw REP cycle lost its actual pipeline record"); }
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const auto &phase = *cycle.pipeline;
  program_phases(trace, phase, sample, clock);
  memory_phases(trace, phase, sample, clock);
  trace.expected = expected_state(trace, sample, clock);
  unsigned stores = 0;
  for (unsigned index = 0; index < sample.count; ++index) {
    if (clock >= times(index, sample).e1) { ++stores; }
  }
  cells(machine, sample, stores);
}

inline DspFetchCycleResult
advance(DspFetchRun &run, DspMachine &machine, Trace &trace, const Case sample) {
  const auto fetched = machine.reference_fetch();
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const bool issued_body = cycle.issued_address == body;
  const bool fetched_body = cycle.bus.fetch && fetched.pending_address == body;
  require(issued_body == (cycle.completed_cycles == 3) &&
              fetched_body == (cycle.completed_cycles == 4),
          "The body has exactly the independently scheduled original F1/F2 receipt");
  if (issued_body) { ++trace.f1; }
  if (fetched_body) {
    ++trace.f2;
    if (sample.mutate) { machine.bus().write({.address = body, .value = illegal_opcode}); }
  }
  observe(trace, machine, cycle, sample);
  if (machine.cpu().debug_state() != trace.expected) {
    std::cerr << "period=" << cycle.completed_cycles << " opcode=" << hex(opcode(sample), 6)
              << " count=" << sample.count << " source=" << sample.source << '\n';
  }
  require(machine.cpu().debug_state() == trace.expected,
          "Every raw REP period preserves complete CPU,CCR,LC,source,AGU and native counter "
          "expectations");
  return cycle;
}

inline void completed(const Trace &trace, const Case sample, const DspFetchCycleResult &cycle) {
  require(trace.bodies == sample.count && trace.stores == sample.count &&
              trace.addresses == sample.count && trace.decoded == sample.count && trace.f1 == 1 &&
              trace.f2 == 1,
          "All genuine raw REP bodies execute from one owned F1/F2 receipt");
  require(trace.prefix_decode && trace.first_body_decode && trace.last_body_e2 &&
              *trace.first_body_decode == *trace.prefix_decode + 5 &&
              cycle.completed_cycles == *trace.last_body_e2 + 1,
          "Actual raw REP setup and final-body/successor overlap retain nominal timing");
}

struct Saved {
  DspFetchCheckpoint checkpoint;
  Trace trace;
};

inline void finish(DspFetchRun &run, DspMachine &machine, Trace &trace, const Case sample) {
  for (unsigned index = 0; index < sample.count * 2 + bound_slack; ++index) {
    const auto cycle = advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != following + 1) { continue; }
    completed(trace, sample, cycle);
    cells(machine, sample, sample.count);
    return;
  }
  throw Fault("Raw REP stream exceeded its explicit count-derived bound");
}

inline unsigned one(const Case sample, const bool snapshots = false) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  Trace trace{.expected = before, .initial = before};
  auto contract = fetch_clock::contract();
  contract.execution = execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  std::vector<Saved> cuts;
  for (unsigned index = 0; index < sample.count * 2 + bound_slack; ++index) {
    if (snapshots) { cuts.push_back({.checkpoint = run.checkpoint(machine), .trace = trace}); }
    const auto cycle = advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != following + 1) { continue; }
    completed(trace, sample, cycle);
    cells(machine, sample, sample.count);
    const auto expected = machine.cpu().debug_state();
    for (const auto &saved : cuts) {
      for (unsigned replay = 0; replay < 2; ++replay) {
        run.restore(saved.checkpoint, machine);
        auto tail = saved.trace;
        finish(run, machine, tail, sample);
        require(machine.cpu().debug_state() == expected,
                "Every trusted raw REP snapshot reproduces the full native endpoint");
      }
    }
    return static_cast<unsigned>(cuts.size());
  }
  throw Fault("Raw REP stream exceeded its explicit count-derived bound");
}

inline void refused(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  Trace trace{.expected = before, .initial = before};
  auto contract = fetch_clock::contract();
  contract.execution = execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const auto error =
      checkpoint::rejects([&] { finish(run, machine, trace, sample); },
                          "Expected the current missing raw REP instruction admission");
  const bool expected = error.contains("Unqualified REP body read P:") ||
                        error.contains("Unqualified repeated Decode read P:") ||
                        error.contains("Unqualified automatic DSP decode P:") ||
                        error.contains("Unqualified native program-fetch control opcode");
  require(expected && error.contains("pc=") && error.contains("omr=") && run.faulted() &&
              trace.bodies == 0 && trace.stores == 0 && trace.addresses == 0 &&
              machine.cpu().debug_state() == trace.expected,
          "Old raw REP admission fails visibly before body/AGU effects with a complete preserved "
          "prefix");
  cells(machine, sample, 0);
  std::cout << "Baseline raw REP refusal: " << error << '\n';
}
}
