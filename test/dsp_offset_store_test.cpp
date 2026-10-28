// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_register_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace raw = repeat_register;
constexpr unsigned producer_pc = raw::entry + 4;
constexpr unsigned source_address = 0x410;
constexpr unsigned n4_field = 28;
constexpr unsigned expected_streams = 256;
constexpr unsigned n4_short = 0x3c0300;
constexpr unsigned n4_register = 0x20bc00;
constexpr unsigned n4_long = 0x74f400;
constexpr unsigned n4_memory = 0x74e000;

enum class Writer : std::uint8_t { none, immediate, transfer, long_immediate, memory };

struct Case {
  raw::Case store;
  Writer writer = Writer::none;
  unsigned old_source = 0, gap = 0;

  [[nodiscard]] unsigned words() const {
    if (writer == Writer::none) { return 0; }
    return writer == Writer::long_immediate ? 2U : 1U;
  }

  [[nodiscard]] unsigned body() const { return producer_pc + words() + gap; }

  [[nodiscard]] unsigned after() const { return body() + 1; }

  [[nodiscard]] unsigned decode() const { return 7 + words() + gap; }

  [[nodiscard]] unsigned data_clock() const { return decode() + 2 + raw::body_clocks(store); }

  [[nodiscard]] unsigned end_clock() const { return data_clock() + 2; }

  [[nodiscard]] unsigned value() const { return writer == Writer::none ? raw::value(store) : 3U; }
};

unsigned writer_opcode(const Writer writer) {
  switch (writer) {
    case Writer::none: return 0;
    case Writer::immediate: return n4_short;
    case Writer::transfer: return n4_register;
    case Writer::long_immediate: return n4_long;
    case Writer::memory: return n4_memory;
    default: throw Fault("Undefined own standalone N4 producer");
  }
}

void prepare(DspMachine &machine, const Case sample) {
  DspProgram program;
  program.immediate("omr", raw::ordinary_omr);
  program.immediate("lc", raw::old_lc);
  for (unsigned index = 0; index < raw::data_values.size(); ++index) {
    const auto value =
        index == 1 && sample.writer == Writer::transfer ? 3U : raw::data_values.at(index);
    program.immediate(raw::data_names.at(index), value);
  }
  program.immediate("a0", raw::a0_seed);
  program.immediate("a1", 1);
  program.immediate("a2", raw::a2_seed);
  program.immediate("b0", raw::b0_seed);
  program.immediate("b1", raw::b1_seed);
  program.immediate("b2", raw::b2_seed);
  program.immediate("r0", source_address);
  program.immediate("n0", 0);
  program.immediate("m0", raw::linear);
  const auto reg = std::to_string(sample.store.reg);
  program.immediate("r" + reg, raw::data);
  program.immediate("n" + reg, raw::offset);
  program.immediate("m" + reg, raw::linear);
  const auto source = std::to_string(sample.store.source - raw::offset_first);
  program.immediate("n" + source,
                    sample.writer == Writer::none ? sample.value() : sample.old_source);
  program.immediate("sr", masked_status | sample.store.flags);
  for (unsigned index = 0; index < 5; ++index) { program.word(0); }
  program.word(raw::jump);
  program.word(raw::entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, raw::entry);
  for (unsigned address = raw::entry; address < raw::entry + 32; ++address) {
    machine.bus().write({.address = address});
  }
  if (sample.writer != Writer::none) {
    machine.bus().write({.address = producer_pc, .value = writer_opcode(sample.writer)});
    if (sample.words() == 2) { machine.bus().write({.address = producer_pc + 1, .value = 3}); }
  }
  machine.bus().write({.address = sample.body(), .value = raw::opcode(sample.store)});
  const auto [first, last] = raw::range(sample.store);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first; address <= last; ++address) {
      machine.bus().write({.space = space, .address = address, .value = raw::canary});
    }
  }
  machine.bus().write({.space = DspSpace::x, .address = source_address, .value = 3});
  const auto state = machine.cpu().state();
  if (sample.writer == Writer::none) { raw::validate_seeds(state, sample.store); }
  const auto input = sample.writer == Writer::none ? sample.value() : sample.old_source;
  require(state.pc == raw::entry && state.lc == raw::old_lc &&
              state.r.at(sample.store.reg) == raw::data &&
              state.n.at(sample.store.source - raw::offset_first) == input &&
              state.sr == (masked_status | sample.store.flags),
          "Native setup supplies the exact standalone source/AGU/LC/CCR inputs");
  static_cast<void>(machine.begin_reference_instruction(raw::execution));
}

DspDebugState expected_state(const DspDebugState &before, const Case sample, const unsigned clock) {
  auto expected = before;
  for (unsigned index = 0; index < 4; ++index) {
    if (clock < 7 + index) { continue; }
    ++expected.native_instructions;
    ++expected.native_cycles;
    expected.cpu.pc = raw::entry + index + 1;
  }
  if (sample.writer != Writer::none) {
    if (clock >= 10) { expected.cpu.n.at(4) = 3; }
    if (clock >= 11) {
      ++expected.native_instructions;
      expected.native_cycles += sample.words();
      expected.cpu.pc = producer_pc + sample.words();
    }
  }
  for (unsigned index = 0; index < sample.gap; ++index) {
    if (clock < 11 + sample.words() + index) { continue; }
    ++expected.native_instructions;
    ++expected.native_cycles;
    expected.cpu.pc = producer_pc + sample.words() + index + 1;
  }
  if (clock >= sample.decode() + 2) {
    expected.cpu.r.at(sample.store.reg) = raw::pointer(sample.store, 1);
  }
  if (clock >= sample.data_clock() + 1) {
    ++expected.native_instructions;
    expected.native_cycles += raw::body_clocks(sample.store);
    expected.cpu.pc = sample.after();
  }
  if (clock >= sample.end_clock()) {
    ++expected.native_instructions;
    ++expected.native_cycles;
    expected.cpu.pc = sample.after() + 1;
  }
  return expected;
}

void cells(DspMachine &machine, const Case sample, const unsigned clock) {
  const auto [first, last] = raw::range(sample.store);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = first; address <= last; ++address) {
      const bool stored = space == raw::space(sample.store) &&
                          address == raw::address(sample.store, 0) && clock >= sample.data_clock();
      require(machine.bus().read({.space = space, .address = address}) ==
                  (stored ? sample.value() : raw::canary),
              "Every standalone period checks partial RAM and the same opposite-space range");
    }
  }
  require(machine.bus().read({.space = DspSpace::x, .address = source_address}) == 3,
          "Standalone writers retain the independently seeded load source cell");
}

void address_phases(const DspPipelineCycle &phase, const Case sample, const unsigned clock) {
  const bool writer_ag1 = sample.writer == Writer::memory && clock == 8;
  const bool writer_ag2 = sample.writer == Writer::memory && clock == 9;
  const bool body_ag1 = clock == sample.decode() + 1;
  const bool body_ag2 = clock == sample.decode() + 2;
  require(phase.ag1_address.has_value() == (writer_ag1 || body_ag1) &&
              phase.ag2_address.has_value() == (writer_ag2 || body_ag2),
          "Standalone writer and store AG1/AG2 effects have fixed independent periods");
  if (phase.ag1_address) {
    const auto &stage = *phase.ag1_address;
    const bool update = body_ag1 && sample.store.mode != 4 && sample.store.mode != 5;
    require(!stage.published && stage.word.address == (writer_ag1 ? producer_pc : sample.body()) &&
                stage.pointer_update == update &&
                stage.update.address ==
                    (writer_ag1 ? source_address : raw::address(sample.store, 0)) &&
                stage.update.before == (writer_ag1 ? source_address : raw::data) &&
                stage.update.value == (writer_ag1 ? source_address : raw::pointer(sample.store, 1)),
            "Scheduled standalone AG1 retains independent address and pointer expectations");
  }
  if (phase.ag2_address) {
    const auto &stage = *phase.ag2_address;
    require(stage.published && stage.word.address == (writer_ag2 ? producer_pc : sample.body()) &&
                stage.update.address ==
                    (writer_ag2 ? source_address : raw::address(sample.store, 0)),
            "Scheduled standalone AG2 publishes its exact native instruction address");
  }
}

void data_phases(const DspPipelineCycle &phase, const Case sample, const unsigned clock) {
  const bool writer_read = sample.writer == Writer::memory && clock == 10;
  const bool body_write = clock == sample.data_clock();
  const bool transfer =
      sample.writer != Writer::none && sample.writer != Writer::memory && clock == 10;
  require(phase.memory.has_value() == (writer_read || body_write) &&
              phase.transfer.has_value() == transfer,
          "Standalone source publication and raw data access have fixed independent E1 periods");
  if (phase.transfer) {
    require(phase.transfer->pc == producer_pc, "Only the real N4 writer transfers");
  }
  if (phase.memory) {
    const auto &access = phase.memory->data;
    require(!phase.memory->additional_data &&
                access.kind ==
                    (writer_read ? DspNativeAccessKind::read : DspNativeAccessKind::write) &&
                access.target.space == (writer_read ? DspSpace::x : raw::space(sample.store)) &&
                access.target.address ==
                    (writer_read ? source_address : raw::address(sample.store, 0)) &&
                access.value == (writer_read ? 3U : sample.value()),
            "Standalone E1 accesses retain independently expected raw bits, space and address");
  }
}

void program_phases(const DspPipelineCycle &phase, const Case sample, const unsigned clock) {
  std::optional<unsigned> retired = std::nullopt;
  for (unsigned index = 0; index < 4; ++index) {
    if (clock == 7 + index) { retired = raw::entry + index; }
  }
  if (sample.writer != Writer::none && clock == 11) { retired = producer_pc; }
  for (unsigned index = 0; index < sample.gap; ++index) {
    if (clock == 11 + sample.words() + index) { retired = producer_pc + sample.words() + index; }
  }
  if (clock == sample.data_clock() + 1) { retired = sample.body(); }
  if (clock == sample.end_clock()) { retired = sample.after(); }
  const bool body_decoded =
      phase.decode_accepted && phase.decode && phase.decode->address == sample.body();
  const bool held = raw::body_clocks(sample.store) == 2 && clock == sample.data_clock() - 1;
  require(body_decoded == (clock == sample.decode()) &&
              phase.retirement_published == retired.has_value() && !phase.memory_source_stall &&
              phase.advanced != held,
          "Standalone Decode/E2/latency obey fixed expectations without an extra N-source hold");
  if (retired) {
    require(phase.execute2 && phase.execute2->address == *retired,
            "Every standalone E2 has its independently scheduled real instruction owner");
  }
}

void one(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  auto contract = fetch_clock::contract();
  contract.execution = raw::execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  for (unsigned clock = 1; clock <= sample.end_clock(); ++clock) {
    const auto fetched = machine.reference_fetch();
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    if (!cycle.pipeline) { throw Fault("Standalone N store lost its actual pipeline state"); }
    require(cycle.completed_cycles == clock, "Each explicit input period advances exactly once");
    require((cycle.issued_address == sample.body()) == (clock == sample.decode() - 2) &&
                (cycle.bus.fetch && fetched.pending_address == sample.body()) ==
                    (clock == sample.decode() - 1),
            "Standalone body F1/F2 retain their independently scheduled unique occurrence");
    program_phases(*cycle.pipeline, sample, clock);
    address_phases(*cycle.pipeline, sample, clock);
    data_phases(*cycle.pipeline, sample, clock);
    cells(machine, sample, clock);
    require(machine.cpu().state().lc == raw::old_lc &&
                machine.cpu().debug_state() == expected_state(before, sample, clock),
            "Every standalone period retains independent complete CPU/CCR state and unchanged LC "
            "in the REP-free program");
  }
}

void address_modes(unsigned &streams) {
  constexpr std::array modes{0U, 1U, 2U, 3U, 4U, 5U, 7U};
  for (unsigned source = raw::offset_first; source <= raw::offset_last; ++source) {
    for (const unsigned mode : modes) {
      for (const unsigned reg : {1U, 3U}) {
        for (const bool y : {false, true}) {
          one({.store = {.source = source, .mode = mode, .reg = reg, .count = 1, .y = y}});
          ++streams;
        }
      }
    }
  }
}

void source_writers(unsigned &streams) {
  constexpr std::array writers{
      Writer::immediate, Writer::transfer, Writer::long_immediate, Writer::memory};
  for (const auto writer : writers) {
    for (unsigned old_source = 0; old_source < 2; ++old_source) {
      for (unsigned gap = 0; gap < 4; ++gap) {
        one({.store = {.source = n4_field, .reg = 1, .count = 1},
             .writer = writer,
             .old_source = old_source,
             .gap = gap});
        ++streams;
      }
    }
  }
}
}

int main() {
  try {
    unsigned streams = 0;
    address_modes(streams);
    source_writers(streams);
    require(streams == expected_streams, "Every standalone N-store and writer control executes");
    std::cout << streams << " standalone N-store/address/source-writer streams passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
