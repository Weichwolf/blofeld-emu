// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_movem_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned register_count = 46;
constexpr unsigned ram = 0x100;
constexpr unsigned start = 0x1fa;
constexpr unsigned move_omr = 0x05f43a;
constexpr unsigned move_sz = 0x05f438;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned seed_words = 54;
constexpr unsigned seed_instructions = 32;
constexpr unsigned maximum_points = 2048;
constexpr unsigned expected_cases = 35;
namespace catalogue = blofeld::test::movem;
using Registers = catalogue::Registers;
using StackWords = catalogue::StackWords;
using Cell = catalogue::Cell;
using Point = catalogue::Point;
using Row = catalogue::Row;

Registers registers(const DspDebugState &state) {
  const auto &cpu = state.cpu;
  Registers result{};
  unsigned index = 0;
  const auto append = [&](const std::uint64_t value) {
    result.at(index++) = static_cast<unsigned>(value & word_mask);
  };
  append(cpu.x >> word_bits);
  append(cpu.x);
  append(cpu.y >> word_bits);
  append(cpu.y);
  for (const auto value : {cpu.a, cpu.b}) {
    append(value >> data_bits);
    append(value >> word_bits);
    append(value);
  }
  for (const auto value :
       {cpu.pc, cpu.sr, cpu.omr, cpu.la, cpu.lc, cpu.sp, state.ep, state.sz, state.sc, cpu.vba}) {
    append(value);
  }
  for (const auto &values : {cpu.r, cpu.n, cpu.m}) {
    for (const auto value : values) { append(value); }
  }
  append(cpu.stack_pc);
  append(cpu.stack_sr);
  require(index == register_count, "Complete vendor register order");
  return result;
}

void verify(const DspDebugState &actual,
            const Registers &expected,
            const StackWords &stack,
            const std::string &where) {
  const auto values = registers(actual);
  for (unsigned index = 0; index < register_count; ++index) {
    require(values.at(index) == expected.at(index),
            where + " register=" + std::to_string(index) + " actual=" + hex(values.at(index), 6) +
                " expected=" + hex(expected.at(index), 6));
  }
  for (std::size_t index = 0; index < actual.stack_high.size(); ++index) {
    require(actual.stack_high.at(index) == stack.at(2 * index) &&
                actual.stack_low.at(index) == stack.at((2 * index) + 1),
            where + " preserves all16 explicitly predicted stack cells");
  }
}

DspDebugState
prepare(DspMachine &machine, const std::vector<unsigned> &words, const std::vector<Cell> &cells) {
  DspProgram image;
  image.word(move_omr);
  image.word(ordinary_omr);
  image.word(move_sz);
  image.word(0);
  for (unsigned index = 0; index < seed_words; ++index) { image.word(words.at(index)); }
  image.install(machine.bus());
  for (std::size_t index = 0; index < words.size(); ++index) {
    machine.bus().write({.address = ram + static_cast<unsigned>(index), .value = words.at(index)});
  }
  for (const auto cell : cells) {
    machine.bus().write({.space = cell.space, .address = cell.address, .value = cell.value});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto initial = machine.cpu().debug_state();
  require(initial.cpu.pc == start && initial.native_instructions == seed_instructions,
          "Genuine own ROM preamble reaches the independent RAM window");
  return initial;
}

struct Early {
  std::size_t owner;
  unsigned field, value;
};

struct Oracle {
  std::vector<Row> rows;
  Registers initial;
  std::vector<Early> early;
  std::size_t retired = 0, issued = 0, addressed = 0, operands = 0;
  std::uint64_t initial_cycles = 0, cycles = 0;
  unsigned program_holds = 0;
};

void check_word(const std::optional<DspProgramWord> actual, const Point point) {
  require(actual && actual->address == point.pc && actual->value == point.opcode,
          "Actual stage owns the independent expected instruction occurrence");
}

void address_kind(const DspAddressStageView &actual, const Point point) {
  constexpr unsigned instruction_mask = 0xff0000;
  constexpr unsigned movem_prefix = 0x070000;
  constexpr unsigned effective_address_bit = 0x4000;
  const bool short_address = (point.opcode & instruction_mask) == movem_prefix &&
                             (point.opcode & effective_address_bit) == 0;
  const bool long_address = point.words == 2;
  auto kind = DspAddressStageKind::register_agu;
  if (short_address) { kind = DspAddressStageKind::absolute_short; }
  if (long_address) { kind = DspAddressStageKind::absolute_long; }
  const bool increment = !short_address && !long_address && ((point.opcode >> 11U) & 7U) == 3;
  require(actual.kind == kind && actual.pointer_update == increment &&
              actual.extension.has_value() == long_address,
          "Independent address kind and actual extension/AGU publication contract");
  if (actual.extension) {
    require(actual.extension->address == point.pc + 1 && actual.extension->value == point.extension,
            "Long P operand uses its genuine captured F2 extension");
  }
}

void address_footprint(const DspAddressStageView &actual, const Row &expected) {
  const auto point = expected.point;
  require(actual.published && actual.update.space == point.memory_space &&
              actual.update.address == point.address && actual.update.pc == point.pc &&
              actual.update.omr == expected.state.at(12) && actual.data_access_count == 1 &&
              actual.data_accesses[0].target.space == point.memory_space &&
              actual.data_accesses[0].target.address == point.address &&
              actual.data_accesses[0].operation ==
                  (point.kind == 1 ? DspMemoryOperation::read : DspMemoryOperation::write) &&
              !actual.secondary,
          "Independent actual AG2 data direction/address/context and one-operand footprint");
}

void address(Oracle &oracle, const DspAddressStageView &actual) {
  while (oracle.addressed < oracle.rows.size() &&
         oracle.rows.at(oracle.addressed).point.kind == 0) {
    ++oracle.addressed;
  }
  require(oracle.addressed < oracle.rows.size(), "No extra AG2 operand");
  const auto owner = oracle.addressed++;
  const auto &expected = oracle.rows.at(owner);
  const auto point = expected.point;
  check_word(actual.word, point);
  address_kind(actual, point);
  address_footprint(actual, expected);
  if (actual.pointer_update) {
    constexpr unsigned first_r = 20;
    const auto field = first_r + ((point.opcode >> 8U) & 7U);
    const auto before =
        owner == 0 ? oracle.initial.at(field) : oracle.rows.at(owner - 1).state.at(field);
    require(actual.update.index == field - first_r && actual.update.before == before &&
                actual.update.value == ((before + 1) & word_mask) &&
                actual.update.value == expected.state.at(field),
            "Real AG2 publishes the independently expected postincrement only");
    oracle.early.push_back({.owner = owner, .field = field, .value = expected.state.at(field)});
  }
}

void memory(Oracle &oracle,
            const DspMemoryStageResult &actual,
            const std::size_t owner,
            std::vector<Cell> &cells,
            std::vector<unsigned> &code) {
  const auto &expected = oracle.rows.at(owner);
  const auto point = expected.point;
  check_word(actual.address.word, point);
  require(point.kind != 0 && !actual.additional_data &&
              actual.data.kind ==
                  (point.kind == 1 ? DspNativeAccessKind::read : DspNativeAccessKind::write) &&
              actual.data.target.space == point.memory_space &&
              actual.data.target.address == point.address && actual.data.value == point.value &&
              actual.data.pc == point.pc && actual.data.omr == expected.state.at(12),
          "One genuine native data operand matches independent raw value/address/context");
  if (point.kind == 1) {
    constexpr unsigned register_mask = 63;
    const auto destination = point.opcode & register_mask;
    require(destination == 4 || destination == 5, "Independent raw X0/X1 load encoding");
    oracle.early.push_back(
        {.owner = owner, .field = destination == 4 ? 1U : 0U, .value = point.value});
    require(actual.move.has_value(), "Actual native load publishes its raw destination");
  } else {
    require(!actual.move, "Native store fabricates no load publication");
    for (auto &cell : cells) {
      if (cell.space == point.memory_space && cell.address == point.address) {
        cell.value = point.value;
      }
    }
    if (point.memory_space == DspSpace::p && point.address >= ram &&
        point.address - ram < code.size()) {
      code.at(point.address - ram) = point.value;
    }
  }
  ++oracle.operands;
}

void execute(Oracle &oracle,
             const DspPipelineCycle &phase,
             std::vector<Cell> &cells,
             std::vector<unsigned> &code) {
  if (oracle.issued == oracle.rows.size()) {
    const auto expected_pc = oracle.rows.back().state.at(10);
    require(phase.execute1 && phase.execute1->address == expected_pc &&
                phase.execute1->value == 0 && !phase.memory && !phase.transfer,
            "One younger literal padding NOP may reach E1 at the last observed E2 boundary");
    return;
  }
  const auto owner = oracle.issued++;
  const auto &expected = oracle.rows.at(owner);
  check_word(phase.execute1, expected.point);
  if (expected.point.kind != 0) {
    if (!phase.memory) { throw Fault("Every independent data operation must reach real E1"); }
    memory(oracle, *phase.memory, owner, cells, code);
  } else {
    require(!phase.memory, "Ordinary native operation creates no data operand");
    constexpr unsigned immediate_r7 = 0x67f400;
    constexpr unsigned parallel_r3 = 0x21b359;
    if (expected.point.opcode == immediate_r7 || expected.point.opcode == parallel_r3) {
      constexpr unsigned r7_field = 27;
      constexpr unsigned r3_field = 23;
      const unsigned field = expected.point.opcode == immediate_r7 ? r7_field : r3_field;
      require(phase.transfer.has_value(), "Real register transfer publishes before E2");
      oracle.early.push_back({.owner = owner, .field = field, .value = expected.state.at(field)});
    }
  }
}

void update(Oracle &oracle,
            const DspPipelineCycle &phase,
            std::vector<Cell> &cells,
            std::vector<unsigned> &code) {
  if (phase.retirement_published) {
    const auto &expected = oracle.rows.at(oracle.retired++);
    check_word(phase.execute2, expected.point);
    oracle.cycles += expected.point.cost;
  }
  if (phase.ag2_address) { address(oracle, *phase.ag2_address); }
  if (phase.execution_captured) { execute(oracle, phase, cells, code); }
  if (phase.execute1 && !phase.execution_captured && !phase.memory_source_stall) {
    const auto opcode = phase.execute1->value;
    constexpr unsigned movem_mask = 0xff0000;
    constexpr unsigned movem_prefix = 0x070000;
    if ((opcode & movem_mask) == movem_prefix) { ++oracle.program_holds; }
  }
}

void check(const DspMachine &machine,
           const Oracle &oracle,
           const unsigned period,
           const std::string &name) {
  auto expected = oracle.retired == 0 ? oracle.initial : oracle.rows.at(oracle.retired - 1).state;
  const auto stack = oracle.retired == 0 ? StackWords{} : oracle.rows.at(oracle.retired - 1).stack;
  for (const auto early : oracle.early) {
    if (early.owner >= oracle.retired) { expected.at(early.field) = early.value; }
  }
  const auto actual = machine.cpu().debug_state();
  verify(actual, expected, stack, name + " period=" + std::to_string(period));
  require(actual.native_instructions == seed_instructions + oracle.retired &&
              actual.native_cycles == oracle.initial_cycles + oracle.cycles,
          "Full nominal table counters retain independently predicted retirements");
}

void storage(DspMachine &machine,
             const std::vector<Cell> &cells,
             const std::vector<unsigned> &code) {
  for (const auto cell : cells) {
    require(machine.bus().read({.space = cell.space, .address = cell.address}) == cell.value,
            "Every monitored P/X cell matches only declared real E1 writes");
  }
  for (std::size_t index = 0; index < code.size(); ++index) {
    require(machine.bus().read({.address = ram + static_cast<unsigned>(index)}) == code.at(index),
            "Whole own code retains only independently declared native patches");
  }
}

void advance(DspFetchRun &run,
             DspMachine &machine,
             Oracle &oracle,
             std::vector<Cell> &cells,
             std::vector<unsigned> &code,
             const std::string &name) {
  const auto cycle =
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
  if (!cycle.pipeline) { throw Fault("Actual automatic pipeline must own every period"); }
  update(oracle, *cycle.pipeline, cells, code);
  const auto period = static_cast<unsigned>(run.completed_cycles());
  check(machine, oracle, period, name);
  storage(machine, cells, code);
  require(period < maximum_points, "Bounded automatic window must terminate");
}

void finish(DspFetchRun &run,
            DspMachine &machine,
            Oracle &oracle,
            std::vector<Cell> &cells,
            std::vector<unsigned> &code,
            const unsigned expected_periods,
            const std::string &name) {
  while (oracle.retired != oracle.rows.size()) { advance(run, machine, oracle, cells, code, name); }
  require(run.completed_cycles() == expected_periods,
          name + " independent whole-window clock total actual=" +
              std::to_string(run.completed_cycles()) +
              " expected=" + std::to_string(expected_periods));
  unsigned expected_holds = 0;
  for (const auto &expected : oracle.rows) {
    constexpr unsigned movem_mask = 0xff0000;
    constexpr unsigned movem_prefix = 0x070000;
    if ((expected.point.opcode & movem_mask) == movem_prefix) { expected_holds += 5; }
  }
  require(oracle.program_holds == expected_holds,
          "Each real MOVEM retains five reference E1 hold periods");
}

bool snapshot_case(const std::string &name) {
  constexpr std::array<std::string_view, 10> selected{"load-r7-omr300-plain-gap0-call0",
                                                      "store-r7-omr300-plain-gap0-call0",
                                                      "absolute-load-r7-omr300-plain-gap0-call0",
                                                      "absolute-store-r7-omr300-plain-gap0-call0",
                                                      "store-r7-omr300-alu-gap0-call0",
                                                      "store-r7-omr300-address-gap0-call0",
                                                      "loop-store-r7-omr300-plain-gap0-call1",
                                                      "patch-loop-r7-omr300-plain-gap0-call1",
                                                      "loop-store-r7-omr600389-plain-gap0-call1",
                                                      "patch-loop-r7-omr600389-plain-gap0-call1"};
  return std::ranges::contains(selected, std::string_view{name});
}

void snapshots(DspFetchRun &run,
               DspMachine &machine,
               Oracle &oracle,
               std::vector<Cell> &cells,
               std::vector<unsigned> &code,
               const unsigned expected_periods,
               const std::string &name) {
  unsigned cuts = 0;
  for (;;) {
    const auto checkpoint = run.checkpoint(machine);
    const auto prefix = oracle;
    const auto prefix_cells = cells;
    const auto prefix_code = code;
    const auto reset = [&] {
      run.restore(checkpoint, machine);
      oracle = prefix;
      cells = prefix_cells;
      code = prefix_code;
      check(machine, oracle, static_cast<unsigned>(run.completed_cycles()), name);
      storage(machine, cells, code);
    };
    for (unsigned suffix = 0; suffix < 2; ++suffix) {
      reset();
      finish(run, machine, oracle, cells, code, expected_periods, name);
    }
    ++cuts;
    reset();
    if (oracle.retired == oracle.rows.size()) {
      require(cuts == expected_periods + 1,
              "Every completed-period boundary has two checked suffixes");
      std::cout << "snapshot " << name << " cuts=" << cuts << " suffixes=" << 2 * cuts
                << " full-state/reference-clock passed\n"
                << std::flush;
      return;
    }
    advance(run, machine, oracle, cells, code, name);
  }
}

void one(const catalogue::Case sample, const bool snapshot_mode) {
  const auto generated = catalogue::window(sample);
  const auto name = catalogue::name(sample);
  auto code = generated.program.words;
  auto cells = generated.program.cells;
  Oracle oracle{.rows = generated.rows, .initial = generated.initial, .early = {}};
  oracle.early.reserve(std::size_t{2} * oracle.rows.size());
  const auto expected_periods = generated.periods;
  if (snapshot_mode && !snapshot_case(name)) { return; }
  DspMachine machine;
  const auto initial = prepare(machine, code, cells);
  constexpr unsigned initial_cycles = 60;
  require(initial.native_cycles == initial_cycles, "Independent complete ROM preamble table costs");
  oracle.initial_cycles = initial_cycles;
  check(machine, oracle, 0, name);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  const DspFetchRunInputs contract{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .timing = DspFetchTiming::reference_cycle_order,
                                   .execution = DspReferenceExecution::staged_register_boundaries,
                                   .pipeline = DspFetchPipeline::automatic_memory};
  DspFetchRun run(contract, machine);
  if (snapshot_mode) {
    snapshots(run, machine, oracle, cells, code, expected_periods, name);
    return;
  }
  finish(run, machine, oracle, cells, code, expected_periods, name);
  std::cout << name << " points=" << oracle.retired << " operands=" << oracle.operands
            << " periods=" << run.completed_cycles() << " reference-phase full-state passed\n"
            << std::flush;
}

struct Encoding {
  unsigned mode, index, operand;
  bool load, short_address;
};

void accepted_form(const DspMemoryForm &form, const Encoding encoding, const unsigned opcode) {
  constexpr unsigned x0 = 4;
  require(is_program_memory(form) && !is_program_transfer(form) && !form.peripheral &&
              form.lanes[0].space == DspSpace::p && form.lane_count == 1 && !form.long_pair &&
              form.lanes[0].operation ==
                  (encoding.load ? DspMemoryOperation::read : DspMemoryOperation::write) &&
              form.absolute_short.has_value() == encoding.short_address,
          "MOVEM owns one P operand and never a paired peripheral operand");
  if (form.absolute_short) {
    require(form.absolute_short->address == encoding.index &&
                form.absolute_short->space == DspSpace::p,
            "Literal AA6 owns the short P address");
  } else {
    require(form.lanes[0].mode == encoding.mode && form.lanes[0].reg == encoding.index,
            "Literal MMM/RRR own the effective P address");
  }
  DspMoveFields expected;
  if (encoding.load) { expected.x = static_cast<std::uint8_t>(encoding.operand == x0 ? 1U : 2U); }
  require(decode_move_writeback(opcode) == expected &&
              decode_move_writeback(opcode, DspMoveDecode::nonmemory) == DspMoveFields{},
          "Six-bit raw data destination and memory/nonmemory writeback scopes remain distinct");
}

void decode_guard(const Encoding encoding) {
  constexpr unsigned x0 = 4;
  constexpr unsigned x1 = 5;
  constexpr unsigned a1 = 12;
  constexpr unsigned b1 = 13;
  constexpr unsigned register_prefix = 0x074080;
  constexpr unsigned short_prefix = 0x070000;
  const auto opcode = (encoding.short_address ? short_prefix : register_prefix) |
                      (static_cast<unsigned>(encoding.load) << 15U) | (encoding.index << 8U) |
                      (encoding.short_address ? 0 : encoding.mode << 11U) | encoding.operand;
  const bool data = encoding.operand == x0 || encoding.operand == x1;
  const bool middle = encoding.operand == a1 || encoding.operand == b1;
  const bool operand = data || (middle && !encoding.load);
  const bool address = encoding.short_address || encoding.mode == 3 || encoding.mode == 4 ||
                       (encoding.mode == 6 && encoding.index == 0);
  const auto form = decode_memory_stage(opcode);
  require(form.has_value() == (operand && address),
          "Independent MOVEM register/address admission table");
  bool admitted = false;
  try {
    const auto format = decode_pipeline_word(
        {.address = start, .value = opcode}, ordinary_omr, DspFetchPipeline::automatic_memory);
    admitted = true;
    require(form && format.memory && !format.transfer && !format.branch &&
                format.words == (!encoding.short_address && encoding.mode == 6 ? 2U : 1U),
            "Actual automatic format agrees with literal MOVEM length and data operation");
  } catch (const Fault &error) {
    require(!(operand && address) &&
                std::string(error.what()).contains("Unqualified automatic DSP decode"),
            "Unsupported instruction form refuses with context");
  }
  require(admitted == form.has_value(),
          "No unsupported MOVEM falls through as a register operation");
  if (!form) { return; }
  accepted_form(*form, encoding, opcode);
}

void decode_guards() {
  constexpr unsigned operand_fields = 64;
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned index = 0; index < 8; ++index) {
      for (unsigned operand = 0; operand < operand_fields; ++operand) {
        for (const bool load : {false, true}) {
          decode_guard({.mode = mode,
                        .index = index,
                        .operand = operand,
                        .load = load,
                        .short_address = false});
        }
      }
    }
  }
  constexpr std::array short_addresses{0U, 3U, 60U, 63U};
  for (const auto index : short_addresses) {
    for (unsigned operand = 0; operand < operand_fields; ++operand) {
      for (const bool load : {false, true}) {
        decode_guard(
            {.mode = 0, .index = index, .operand = operand, .load = load, .short_address = true});
      }
    }
  }
  std::cout << "8704 literal MOVEM format/writeback guards passed\n" << std::flush;
}

void refused(DspMachine &machine,
             const std::vector<unsigned> &code,
             const std::vector<Cell> &cells,
             const std::string &name,
             const DspDebugState &initial) {
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  const DspFetchRunInputs contract{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .timing = DspFetchTiming::reference_cycle_order,
                                   .execution = DspReferenceExecution::staged_register_boundaries,
                                   .pipeline = DspFetchPipeline::automatic_memory};
  DspFetchRun run(contract, machine);
  constexpr unsigned bounded_periods = 64;
  constexpr unsigned leading_nops = 6;
  constexpr unsigned operation_pc = 0x200;
  for (unsigned period = 0; period < bounded_periods; ++period) {
    try {
      static_cast<void>(
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
    } catch (const Fault &error) {
      const std::string text = error.what();
      require(
          text.contains("Unqualified automatic P-write to captured/issued instruction write P:" +
                        hex(code.at(operation_pc + 1 - ram), 6)) &&
              text.contains("pc=" + hex(operation_pc, 6)) && text.contains("omr="),
          "Actual P-write guard refuses a captured instruction/extension with address/PC/OMR");
      const auto actual = machine.cpu().debug_state();
      const auto retired = actual.native_instructions - initial.native_instructions;
      require(retired == leading_nops,
              "Exactly six genuine preceding NOPs retire before P-write refusal");
      auto expected = initial;
      expected.cpu.pc += static_cast<unsigned>(retired);
      expected.native_instructions += retired;
      expected.native_cycles += retired;
      require(actual == expected,
              "Full CPU/counters/stack retain only allowed preceding NOP results");
      for (const auto cell : cells) {
        require(machine.bus().read({.space = cell.space, .address = cell.address}) == cell.value,
                "Automatic P-write refusal has no data operand effect");
      }
      for (std::size_t index = 0; index < code.size(); ++index) {
        require(machine.bus().read({.address = ram + static_cast<unsigned>(index)}) ==
                    code.at(index),
                "Automatic P-write refusal has no program-memory effect");
      }
      bool quarantined = false;
      try {
        static_cast<void>(run.checkpoint(machine));
      } catch (const Fault &) { quarantined = true; }
      require(quarantined, "Faulted automatic execution cannot export a trusted checkpoint");
      std::cout << name << " completed-periods=" << run.completed_cycles()
                << " retired-NOPs=" << retired
                << " captured P-write visibly refused without data effects\n";
      return;
    }
  }
  throw Fault("Expected automatic MOVEM refusal did not occur");
}

void code_guards() {
  const auto generated = catalogue::window({.kind = catalogue::Kind::absolute_store});
  constexpr unsigned operation_pc = 0x200;
  for (unsigned target = operation_pc; target < operation_pc + 3; ++target) {
    auto code = generated.program.words;
    code.at(operation_pc + 1 - ram) = target;
    DspMachine machine;
    const auto initial = prepare(machine, code, generated.program.cells);
    verify(initial, generated.initial, StackWords{}, "Independent guarded-store preconditions");
    refused(machine, code, generated.program.cells, "Captured P target=" + hex(target, 6), initial);
  }
  std::cout << "3 actual captured-P-write refusals and unchanged memories passed\n";
}

}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select MOVEM streams,snapshots or guards");
    const std::string_view mode(arguments[1]);
    require(mode == "streams" || mode == "snapshots" || mode == "guards",
            "Unknown MOVEM test scope");
    if (mode == "guards") {
      decode_guards();
      code_guards();
    } else {
      const auto samples = catalogue::cases();
      require(samples.size() == expected_cases, "Complete independently checked MOVEM catalogue");
      for (const auto sample : samples) { one(sample, mode == "snapshots"); }
      std::cout << (mode == "snapshots" ? "10 automatic MOVEM checkpoint cases passed\n"
                                        : "35 automatic MOVEM reference-phase windows passed\n");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
