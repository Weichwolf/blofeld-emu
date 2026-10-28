// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned entry = 0x100;
constexpr unsigned producer_pc = entry + 4;
constexpr unsigned data = 0x400;
constexpr unsigned source_address = 0x410;
constexpr unsigned canary = 0x654321;
constexpr unsigned source_value = 0xab1234;
constexpr unsigned saved_lc = 0x33;
constexpr unsigned expected_streams = 160;
constexpr unsigned maximum_clocks = 96;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned x0_seed = 0x112233;
constexpr unsigned y0_seed = 0x778899;
constexpr unsigned y1_seed = 0xaabbcc;
constexpr unsigned jump = 0x0af080;
constexpr unsigned rep_a1 = 0x06cc20;
constexpr unsigned rep_three = 0x0603a0;
constexpr unsigned n4_store = 0x745900;
constexpr unsigned extension_shift = 2 * word_bits;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned illegal_opcode = 7;
constexpr std::uint64_t initial_a0 = 0xd4e5f6;
constexpr std::uint64_t initial_a2 = 0x12;
constexpr std::uint64_t initial_b = 0xe93f4a5b6c7d8e;
constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

enum class Kind : std::uint8_t { immediate, transfer, long_immediate, memory, arithmetic };

struct Producer {
  unsigned opcode;
  unsigned part;
  unsigned value;
  Kind kind;
  bool b = false;
};

constexpr std::array producers{
    Producer{.opcode = 0x280100, .part = 0, .value = 1, .kind = Kind::immediate},
    Producer{.opcode = 0x2c0300, .part = 1, .value = 3, .kind = Kind::immediate},
    Producer{.opcode = 0x2ae900, .part = 2, .value = 0xe9, .kind = Kind::immediate},
    Producer{.opcode = 0x20ae00, .part = 3, .value = 3, .kind = Kind::transfer},
    Producer{.opcode = 0x290700, .part = 0, .value = 7, .kind = Kind::immediate, .b = true},
    Producer{.opcode = 0x2d0900, .part = 1, .value = 9, .kind = Kind::immediate, .b = true},
    Producer{.opcode = 0x2b1200, .part = 2, .value = 0x12, .kind = Kind::immediate, .b = true},
    Producer{.opcode = 0x20af00, .part = 3, .value = 3, .kind = Kind::transfer, .b = true},
    Producer{.opcode = 0x54f400, .part = 1, .value = 3, .kind = Kind::long_immediate},
    Producer{.opcode = 0x55f400, .part = 1, .value = 3, .kind = Kind::long_immediate, .b = true},
    Producer{.opcode = 0x50e000, .part = 0, .value = 3, .kind = Kind::memory},
    Producer{.opcode = 0x54e000, .part = 1, .value = 3, .kind = Kind::memory},
    Producer{.opcode = 0x52e000, .part = 2, .value = 3, .kind = Kind::memory},
    Producer{.opcode = 0x55e000, .part = 1, .value = 3, .kind = Kind::memory, .b = true},
    Producer{.opcode = 0x200022, .part = 3, .value = 3, .kind = Kind::arithmetic},
    Producer{.opcode = 0x20002a, .part = 3, .value = 3, .kind = Kind::arithmetic, .b = true}};

struct Case {
  Producer writer;
  unsigned old_count = 3, gap = 0;
  bool register_count = true, mutate = false;

  [[nodiscard]] unsigned words() const { return writer.kind == Kind::long_immediate ? 2U : 1U; }

  [[nodiscard]] unsigned prefix() const { return producer_pc + words() + gap; }

  [[nodiscard]] unsigned body() const { return prefix() + 1; }

  [[nodiscard]] unsigned after() const { return body() + 1; }

  [[nodiscard]] unsigned prefix_decode() const { return 7 + words() + gap; }

  [[nodiscard]] unsigned body_decode() const { return prefix_decode() + 5 + hold(); }

  [[nodiscard]] unsigned hold() const {
    return register_count && !writer.b && words() == 1 && gap == 0 ? 1U : 0U;
  }
};

std::uint64_t result(const std::uint64_t before, const Producer writer) {
  if (writer.part == 3) { return static_cast<std::uint64_t>(writer.value) << word_bits; }
  const auto shift = writer.part * word_bits;
  const std::uint64_t mask = writer.part == 2 ? byte_mask : word_mask;
  return (before & ~(mask << shift)) | (static_cast<std::uint64_t>(writer.value) << shift);
}

void apply(DspState &state, const Producer writer) {
  auto &target = writer.b ? state.b : state.a;
  target = result(target, writer);
  if (writer.kind == Kind::arithmetic) { state.sr = masked_status | 0x10U; }
}

void prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.immediate("lc", saved_lc);
  program.immediate("x0", x0_seed);
  program.immediate("x1", 3);
  program.immediate("y0", y0_seed);
  program.immediate("y1", y1_seed);
  const bool arithmetic_a = sample.writer.kind == Kind::arithmetic && !sample.writer.b;
  const bool arithmetic_b = sample.writer.kind == Kind::arithmetic && sample.writer.b;
  program.immediate("a0", arithmetic_a ? 0U : static_cast<unsigned>(initial_a0));
  program.immediate("a1", sample.old_count);
  program.immediate("a2", arithmetic_a ? 0U : static_cast<unsigned>(initial_a2));
  program.immediate("b0", arithmetic_b ? 0U : static_cast<unsigned>(initial_b & word_mask));
  program.immediate(
      "b1", arithmetic_b ? 6U : static_cast<unsigned>((initial_b >> word_bits) & word_mask));
  program.immediate("b2", arithmetic_b ? 0U : static_cast<unsigned>(initial_b >> extension_shift));
  program.immediate("r0", source_address);
  program.immediate("n0", 0);
  program.immediate("m0", word_mask);
  program.immediate("r1", data);
  program.immediate("n1", 3);
  program.immediate("m1", word_mask);
  program.immediate("n4", source_value);
  program.immediate("sr", masked_status | 1U);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned address = entry; address < entry + 32; ++address) {
    machine.bus().write({.address = address});
  }
  machine.bus().write({.address = producer_pc, .value = sample.writer.opcode});
  if (sample.words() == 2) { machine.bus().write({.address = producer_pc + 1, .value = 3}); }
  machine.bus().write(
      {.address = sample.prefix(), .value = sample.register_count ? rep_a1 : rep_three});
  machine.bus().write({.address = sample.body(), .value = n4_store});
  for (unsigned address = data - 1; address <= data + 3; ++address) {
    machine.bus().write({.space = DspSpace::x, .address = address, .value = canary});
    machine.bus().write({.space = DspSpace::y, .address = address, .value = canary});
  }
  machine.bus().write({.space = DspSpace::x, .address = source_address, .value = 3});
  const auto state = machine.cpu().state();
  const auto expected_a = (arithmetic_a ? 0 : initial_a0 | (initial_a2 << extension_shift)) |
                          (static_cast<std::uint64_t>(sample.old_count) << word_bits);
  const auto expected_b = arithmetic_b ? std::uint64_t{6} << word_bits : initial_b;
  require(state.a == expected_a && state.b == expected_b && state.lc == saved_lc &&
              state.pc == entry && state.sr == (masked_status | 1U),
          "Independent raw accumulator and stale-count seeds precede actual native writers");
  static_cast<void>(machine.begin_reference_instruction(execution));
}

struct Trace {
  DspDebugState expected;
  DspDebugState initial;
  unsigned bodies = 0, reads = 0, stores = 0, holds = 0, body_f1 = 0, body_f2 = 0;
  std::optional<std::uint64_t> prefix_decode = std::nullopt;
  std::optional<std::uint64_t> body_decode = std::nullopt;
};

DspDebugState expected_state(const Trace &trace, const Case sample, const unsigned clock) {
  auto expected = trace.initial;
  for (unsigned index = 0; index < 4; ++index) {
    if (clock < 7 + index) { continue; }
    ++expected.native_instructions;
    ++expected.native_cycles;
    expected.cpu.pc = entry + index + 1;
  }
  const auto publication = sample.writer.kind == Kind::arithmetic ? 11U : 10U;
  if (clock >= publication) { apply(expected.cpu, sample.writer); }
  if (clock >= 11) {
    ++expected.native_instructions;
    expected.native_cycles += sample.words();
    expected.cpu.pc = producer_pc + sample.words();
  }
  for (unsigned index = 0; index < sample.gap; ++index) {
    if (clock < 11 + sample.words() + index) { continue; }
    ++expected.native_instructions;
    ++expected.native_cycles;
    expected.cpu.pc = producer_pc + sample.words() + index + 1;
  }
  if (clock >= sample.prefix_decode() + 4 + sample.hold()) {
    ++expected.native_instructions;
    expected.native_cycles += 5;
    expected.cpu.pc = sample.body();
    expected.cpu.lc = 3;
  }
  for (unsigned index = 0; index < 3; ++index) {
    if (clock >= sample.body_decode() + 2 + index) { expected.cpu.r.at(1) = data + index + 1; }
    if (clock < sample.body_decode() + 4 + index) { continue; }
    ++expected.native_instructions;
    ++expected.native_cycles;
    const auto remaining = 2 - index;
    expected.cpu.pc = remaining == 0 ? sample.after() : sample.body();
    expected.cpu.lc = remaining == 0 ? saved_lc : remaining;
  }
  if (clock >= sample.body_decode() + 7) {
    ++expected.native_instructions;
    ++expected.native_cycles;
    expected.cpu.pc = sample.after() + 1;
  }
  return expected;
}

void diagnose_phases(const Case sample,
                     const unsigned clock,
                     const DspPipelineCycle &phase,
                     const bool aligned) {
  if (aligned) { return; }
  std::cerr << "phase-clock=" << clock << " writer=" << hex(sample.writer.opcode, 6)
            << " gap=" << sample.gap << " register=" << sample.register_count
            << " decode=" << phase.decode_accepted
            << " decode-pc=" << hex(phase.decode.value_or(DspProgramWord{}).address, 6)
            << " expected-prefix-decode=" << sample.prefix_decode()
            << " expected-body-decode=" << sample.body_decode()
            << " retired=" << phase.retirement_published
            << " e2-pc=" << hex(phase.execute2.value_or(DspProgramWord{}).address, 6)
            << " hold=" << phase.memory_source_stall << " advanced=" << phase.advanced << '\n';
}

void program_phases(Trace &trace,
                    const DspPipelineCycle &phase,
                    const Case sample,
                    const unsigned clock) {
  std::optional<unsigned> retired = std::nullopt;
  for (unsigned index = 0; index < 4; ++index) {
    if (clock == 7 + index) { retired = entry + index; }
  }
  if (clock == 11) { retired = producer_pc; }
  for (unsigned index = 0; index < sample.gap; ++index) {
    if (clock == 11 + sample.words() + index) { retired = producer_pc + sample.words() + index; }
  }
  if (clock == sample.prefix_decode() + 4 + sample.hold()) { retired = sample.prefix(); }
  const bool body_due = clock >= sample.body_decode() && clock < sample.body_decode() + 3;
  if (clock >= sample.body_decode() + 4 && clock < sample.body_decode() + 7) {
    retired = sample.body();
  }
  if (clock == sample.body_decode() + 7) { retired = sample.after(); }
  const bool prefix_due = clock == sample.prefix_decode();
  const bool prefix_decoded =
      phase.decode_accepted && phase.decode && phase.decode->address == sample.prefix();
  const bool body_decoded =
      phase.decode_accepted && phase.decode && phase.decode->address == sample.body();
  const bool hold_due = sample.hold() != 0 && clock == 11;
  const bool aligned = prefix_decoded == prefix_due && body_decoded == body_due &&
                       phase.retirement_published == retired.has_value() &&
                       phase.memory_source_stall == hold_due && phase.advanced != hold_due;
  diagnose_phases(sample, clock, phase, aligned);
  require(aligned,
          "Independent cold Decode/E2 positions and the exact source-hold period remain fixed");
  if (retired) {
    require(phase.execute2 && phase.execute2->address == *retired,
            "Every producer/REP E2 has its independently scheduled instruction owner");
    if (*retired == sample.body()) { ++trace.bodies; }
  }
  if (hold_due) {
    ++trace.holds;
    require(phase.execute1 && phase.execute1->address == sample.prefix() &&
                !phase.execution_captured,
            "The scheduled source hold retains REP E1 while its real writer retires");
  }
  if (prefix_due) { trace.prefix_decode = clock; }
  if (body_due && !trace.body_decode) { trace.body_decode = clock; }
}

struct ExpectedAddress {
  unsigned pc, index, before, after, address;
  bool update;
};

ExpectedAddress expected_address(const Case sample, const unsigned clock, const bool first) {
  if (sample.writer.kind == Kind::memory && clock == (first ? 8U : 9U)) {
    return {.pc = producer_pc,
            .index = 0,
            .before = source_address,
            .after = source_address,
            .address = source_address,
            .update = false};
  }
  const auto index = clock - sample.body_decode() - (first ? 1U : 2U);
  return {.pc = sample.body(),
          .index = 1,
          .before = data + index,
          .after = data + index + 1,
          .address = data + index,
          .update = true};
}

void address_phases(const DspPipelineCycle &phase, const Case sample, const unsigned clock) {
  const bool writer_ag1 = sample.writer.kind == Kind::memory && clock == 8;
  const bool writer_ag2 = sample.writer.kind == Kind::memory && clock == 9;
  const bool body_ag1 = clock >= sample.body_decode() + 1 && clock < sample.body_decode() + 4;
  const bool body_ag2 = clock >= sample.body_decode() + 2 && clock < sample.body_decode() + 5;
  require(phase.ag1_address.has_value() == (writer_ag1 || body_ag1) &&
              phase.ag2_address.has_value() == (writer_ag2 || body_ag2),
          "Every writer and body address has an independent AG1/AG2 period");
  if (phase.ag1_address) {
    const auto &stage = *phase.ag1_address;
    const auto expected = expected_address(sample, clock, true);
    require(!stage.published && stage.word.address == expected.pc &&
                stage.pointer_update == expected.update && stage.update.index == expected.index &&
                stage.update.before == expected.before && stage.update.value == expected.after &&
                stage.update.address == expected.address,
            "Fixed AG1 positions retain independent source/target and postincrement addresses");
  }
  if (phase.ag2_address) {
    const auto &stage = *phase.ag2_address;
    const auto expected = expected_address(sample, clock, false);
    require(stage.published && stage.word.address == expected.pc &&
                stage.update.index == expected.index && stage.update.address == expected.address,
            "Fixed AG2 positions publish the independently scheduled native address");
  }
}

void data_phases(Trace &trace,
                 const DspPipelineCycle &phase,
                 const Case sample,
                 const unsigned clock) {
  const bool writer_read = sample.writer.kind == Kind::memory && clock == 10;
  const bool body_write = clock >= sample.body_decode() + 3 && clock < sample.body_decode() + 6;
  const bool transfer =
      sample.writer.kind != Kind::memory && sample.writer.kind != Kind::arithmetic && clock == 10;
  require(phase.memory.has_value() == (writer_read || body_write) &&
              phase.transfer.has_value() == transfer,
          "Every writer and body read/write/transfer has an independent E1 period");
  if (phase.transfer) {
    require(phase.transfer->pc == producer_pc, "Only the real writer publishes at scheduled E1");
  }
  if (phase.memory) {
    const auto &access = phase.memory->data;
    const auto index = body_write ? clock - sample.body_decode() - 3 : 0U;
    require(!phase.memory->additional_data && access.target.space == DspSpace::x &&
                access.pc == (writer_read ? producer_pc : sample.body()) &&
                access.omr == ordinary_omr &&
                access.target.address == (writer_read ? source_address : data + index) &&
                access.value == (writer_read ? 3U : source_value) &&
                access.kind ==
                    (writer_read ? DspNativeAccessKind::read : DspNativeAccessKind::write),
            "Fixed E1 positions retain exact independent native addresses, context and raw data");
    if (writer_read) {
      ++trace.reads;
    } else {
      ++trace.stores;
    }
  }
}

void cells(DspMachine &machine, const Case sample, const unsigned clock) {
  for (unsigned address = data - 1; address <= data + 3; ++address) {
    const bool stored =
        address >= data && address < data + 3 && clock >= sample.body_decode() + 3 + address - data;
    require(
        machine.bus().read({.space = DspSpace::x, .address = address}) ==
                (stored ? source_value : canary) &&
            machine.bus().read({.space = DspSpace::y, .address = address}) == canary,
        "Every period checks the independent partial RAM state and identical opposite-space range");
  }
  require(machine.bus().read({.space = DspSpace::x, .address = source_address}) == 3,
          "Every period retains the untouched independent memory-writer source cell");
}

void advance(DspFetchRun &run, DspMachine &machine, Trace &trace, const Case sample) {
  const auto fetched = machine.reference_fetch();
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const bool issued_body = cycle.issued_address == sample.body();
  const bool fetched_body = cycle.bus.fetch && fetched.pending_address == sample.body();
  require(issued_body == (clock == 6 + sample.words() + sample.gap) &&
              fetched_body == (clock == 7 + sample.words() + sample.gap),
          "Body F1/F2 occur once at independent source/setup-derived clocks");
  if (issued_body) { ++trace.body_f1; }
  if (fetched_body) {
    ++trace.body_f2;
    if (sample.mutate) { machine.bus().write({.address = sample.body(), .value = illegal_opcode}); }
  }
  if (!cycle.pipeline) { throw Fault("A1 producer stream lost its actual phase record"); }
  program_phases(trace, *cycle.pipeline, sample, clock);
  if (sample.hold() != 0 && clock == 11) {
    require(!cycle.issued_address, "Source hold issues no new F1");
  }
  address_phases(*cycle.pipeline, sample, clock);
  data_phases(trace, *cycle.pipeline, sample, clock);
  trace.expected = expected_state(trace, sample, clock);
  cells(machine, sample, clock);
  if (machine.cpu().debug_state() != trace.expected) {
    std::cerr << "period=" << clock << " writer=" << hex(sample.writer.opcode, 6)
              << " gap=" << sample.gap << " register=" << sample.register_count << '\n';
  }
  require(machine.cpu().debug_state() == trace.expected,
          "Every actual producer/REP period matches independent complete CPU/CCR/LC/AGU state");
}

void endpoint(DspMachine &machine, const Trace &trace, const Case sample) {
  require(trace.bodies == 3 && trace.stores == 3 && trace.holds == sample.hold() &&
              trace.reads == (sample.writer.kind == Kind::memory ? 1U : 0U) && trace.body_f1 == 1 &&
              trace.body_f2 == 1,
          "Latest count3 controls every actual body, source hold and unique F1/F2 ownership");
  require(trace.prefix_decode && trace.body_decode &&
              *trace.prefix_decode == sample.prefix_decode() &&
              *trace.body_decode == sample.body_decode(),
          "Complete writer/REP setup matches independently calculated source readiness timing");
  cells(machine, sample, sample.body_decode() + 7);
}

void finish(DspFetchRun &run, DspMachine &machine, Trace &trace, const Case sample) {
  for (unsigned period = 0; period < maximum_clocks; ++period) {
    advance(run, machine, trace, sample);
    if (machine.cpu().state().pc == sample.after() + 1) {
      endpoint(machine, trace, sample);
      return;
    }
  }
  throw Fault("REP A1 source window exceeded its explicit bound");
}

struct Saved {
  DspFetchCheckpoint checkpoint;
  Trace trace;
};

unsigned one(const Case sample, const bool snapshots) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  Trace trace{.expected = before, .initial = before};
  auto contract = fetch_clock::contract();
  contract.execution = execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  std::vector<Saved> cuts;
  for (unsigned period = 0; period < maximum_clocks; ++period) {
    if (snapshots) { cuts.push_back({.checkpoint = run.checkpoint(machine), .trace = trace}); }
    advance(run, machine, trace, sample);
    if (machine.cpu().state().pc != sample.after() + 1) { continue; }
    endpoint(machine, trace, sample);
    const auto expected = machine.cpu().debug_state();
    for (const auto &saved : cuts) {
      for (unsigned replay = 0; replay < 2; ++replay) {
        run.restore(saved.checkpoint, machine);
        auto tail = saved.trace;
        finish(run, machine, tail, sample);
        require(machine.cpu().debug_state() == expected,
                "Every actual producer/REP snapshot reproduces both full native suffixes");
      }
    }
    return static_cast<unsigned>(cuts.size());
  }
  throw Fault("REP A1 source window exceeded its explicit bound");
}

struct Stats {
  unsigned streams = 0, cuts = 0;
};

void variants(const Producer writer, const bool snapshots, Stats &stats) {
  std::vector<unsigned> old_counts{3};
  if (!writer.b && (writer.part == 1 || writer.part == 3)) { old_counts = {0, 1}; }
  if (writer.kind == Kind::arithmetic && !writer.b) { old_counts = {6}; }
  for (const unsigned old_count : old_counts) {
    for (unsigned gap = 0; gap < 4; ++gap) {
      for (const bool register_count : {false, true}) {
        stats.cuts += one({.writer = writer,
                           .old_count = old_count,
                           .gap = gap,
                           .register_count = register_count,
                           .mutate = snapshots},
                          snapshots);
        ++stats.streams;
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--streams";
    if (count > 2 || (mode != "--streams" && mode != "--snapshots")) {
      throw blofeld::Fault("Expected --streams or --snapshots");
    }
    Stats stats;
    for (const auto writer : producers) { variants(writer, mode == "--snapshots", stats); }
    require(stats.streams == expected_streams,
            "Every independent A1 writer/count/gap control executes");
    std::cout << stats.streams << " A1 producer windows passed " << mode << "; cuts=" << stats.cuts
              << " suffixes=" << 2 * stats.cuts << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
