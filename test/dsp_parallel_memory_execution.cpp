// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_address_stage.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_memory_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::parallel_memory {
namespace {
inline constexpr std::uint64_t pair = 0x234567345678;
inline constexpr std::uint64_t first_product = 0x091a2b000000;
inline constexpr std::uint64_t accumulated = 0x291a2b000000;
inline constexpr unsigned first_high = 0x091a2b;
inline constexpr std::uint64_t small_product = 24;
inline constexpr std::uint64_t negative_accumulator = 0xff923456fedcba;
inline constexpr std::uint64_t partial_accumulator = 0xff234567345678;
// FM5.4.1.2: every literal ALU result here has equal bits47/46 in no-scaling mode.
inline constexpr unsigned alu_status = masked_status | unnormalized;

struct Target {
  const char *name;
  std::uint64_t x = pipeline::unit + memory_pipeline::first;
  std::uint64_t y = memory_pipeline::half;
  std::uint64_t a = 0;
  std::uint64_t b = pipeline::unit;
};

constexpr std::array targets{Target{.name = "a10", .a = pair},
                             Target{.name = "b10", .b = pair},
                             Target{.name = "x", .x = pair},
                             Target{.name = "y", .y = pair},
                             Target{.name = "a", .a = pair},
                             Target{.name = "b", .b = pair},
                             Target{.name = "ab", .a = 0x234567000000, .b = 0x345678000000},
                             Target{.name = "ba", .a = 0x345678000000, .b = 0x234567000000}};

struct Words {
  unsigned x = word_x;
  unsigned y = word_y;
};

constexpr Words negative_words{.x = 0x923456, .y = 0xfedcba};
constexpr std::array negative_targets{
    Target{.name = "a10", .a = 0x923456fedcba},
    Target{.name = "b10", .b = 0x923456fedcba},
    Target{.name = "x", .x = 0x923456fedcba},
    Target{.name = "y", .y = 0x923456fedcba},
    Target{.name = "a", .a = 0xff923456fedcba},
    Target{.name = "b", .b = 0xff923456fedcba},
    Target{.name = "ab", .a = 0xff923456000000, .b = 0xfffedcba000000},
    Target{.name = "ba", .a = 0xfffedcba000000, .b = 0xff923456000000}};

void long_load(const Target target, const Words words = {}) {
  DspMachine machine;
  const std::array code{std::string("move l:(r0)+,") + target.name};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = words.x});
  machine.bus().write({.space = DspSpace::y, .address = initial, .value = words.y});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  auto expected = before;
  expected.cpu.r[0] = 0;
  require(machine.cpu().debug_state() == expected, "Long load has one wrapped AG2 pointer update");
  const auto result = run.advance_cycle(machine, fetch_clock::input());
  expected.cpu.x = target.x;
  expected.cpu.y = target.y;
  expected.cpu.a = target.a;
  expected.cpu.b = target.b;
  const auto memory = stages(result).memory;
  require(memory && memory->additional_data && memory->move && !memory->address.secondary &&
              memory->address.data_access_count == 2 && result.bus.core_access_count == 3 &&
              memory->data.value == words.x && memory->additional_data->value == words.y &&
              machine.cpu().debug_state() == expected,
          std::string(target.name) +
              ": native long destination fields publish after both real E1 reads");
  clocks(run, machine, 1);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Actual long result retires without another update");
}

void raw_long_load_preserves_extension(const bool accumulator_b) {
  DspMachine machine;
  const std::string reg = accumulator_b ? "b" : "a";
  const std::array code{"move l:(r0)+," + reg, "move l:(r0)+," + reg + "10"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = negative_words.x});
  machine.bus().write({.space = DspSpace::y, .address = initial, .value = negative_words.y});
  machine.bus().write({.space = DspSpace::x, .address = 0, .value = word_x});
  machine.bus().write({.space = DspSpace::y, .address = 0, .value = word_y});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  auto expected = before;
  expected.cpu.r[0] = 1;
  auto &accumulator = accumulator_b ? expected.cpu.b : expected.cpu.a;
  accumulator = negative_accumulator;
  require(machine.cpu().debug_state() == expected,
          "Full negative long E1 extends the actual sign into A2/B2 before retirement");
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  accumulator = partial_accumulator;
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(stages(cycle).memory && stages(cycle).retirement_published &&
              machine.cpu().debug_state() == expected,
          "Younger positive A10/B10 E1 preserves the negative extension through older full E2");
  clocks(run, machine, 1);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Raw pair retirement preserves the untouched extension and unrelated CPU fields");
}

void long_store(const char *reg, const unsigned x, const unsigned y) {
  DspMachine machine;
  const std::array code{std::string("move ") + reg + ",l:(r0)+"};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  auto expected = before;
  expected.cpu.r[0] = 0;
  require(machine.cpu().debug_state() == expected &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) == x &&
              machine.bus().read({.space = DspSpace::y, .address = initial}) == y,
          std::string(reg) + ": both actual raw long store words commit with one AGU update");
  clocks(run, machine, 1);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Store-only native E2 preserves data fields and flags");
}

void native_parallel_alu() {
  DspMachine machine;
  const std::array<std::string, 4> code{"mpy y0,x0,a x:(r0)+,x0 y:(r4)+,y0",
                                        "mac y0,x0,a x:(r0)+,x1 y:(r4)+,y1",
                                        "mpy y1,x1,b",
                                        "add b,a"};
  prepare(machine, code);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    machine.bus().write({.space = space, .address = initial, .value = memory_pipeline::half});
  }
  machine.bus().write({.space = DspSpace::x, .address = 0, .value = 3});
  machine.bus().write({.space = DspSpace::y, .address = 0, .value = 4});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  require(machine.cpu().state().a == 0 && machine.cpu().state().r[0] == 1 &&
              machine.cpu().state().r[4] == 1 &&
              machine.cpu().state().x == pipeline::unit + memory_pipeline::half &&
              machine.cpu().state().y == memory_pipeline::half,
          "Parallel E1 uses old multiplier operands while real load fields and younger AG2 are "
          "visible");
  constexpr std::array<std::uint64_t, 4> a{
      first_product, accumulated, accumulated, accumulated + small_product};
  constexpr std::array<std::uint64_t, 4> b{
      pipeline::unit, pipeline::unit, small_product, small_product};
  for (unsigned index = 0; index < a.size(); ++index) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    require(stages(cycle).retirement_published && machine.cpu().state().a == a[index] &&
                machine.cpu().state().b == b[index] && machine.cpu().state().sr == alu_status &&
                machine.cpu().state().pc == body + index + 1,
            "Literal parallel MPY/MAC and deferred E2 arithmetic retain their original native "
            "operands");
  }
  require(
      machine.cpu().state().x == (3 * pipeline::unit) + memory_pipeline::half &&
          machine.cpu().state().y == (4 * pipeline::unit) + memory_pipeline::half &&
          machine.cpu().state().sr == alu_status,
      "Final native ALU flags and both XY sibling load fields match independent literal values: "
      "X=" +
          number(machine.cpu().state().x) + " Y=" + number(machine.cpu().state().y) +
          " SR=" + number(machine.cpu().state().sr));
}

void accumulator_store_interlock(const bool transfer) {
  DspMachine machine;
  const std::array<std::string, 3> code{
      transfer ? "move l:(r0)+,a" : "mpy y0,x0,a", "move a10,l:(r0)+", "move x:(r0)+,x0"};
  prepare(machine, code);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    machine.bus().write(
        {.space = space, .address = initial, .value = space == DspSpace::x ? word_x : word_y});
    machine.bus().write({.space = space, .address = 0, .value = memory_pipeline::first});
  }
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = word_x});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto fetch = machine.reference_fetch();
  const auto waiting = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = stages(waiting);
  const auto address = transfer ? 0U : initial;
  const auto value = transfer ? pair : first_product;
  const auto pointer = transfer ? 1U : 0U;
  require(phase.memory_source_stall && phase.retirement_published && !phase.execution_captured &&
              !phase.advanced && waiting.bus.core_access_count == 0 &&
              machine.reference_fetch() == fetch && machine.cpu().state().a == value &&
              machine.cpu().state().r[0] == pointer && run.completed_cycles() == 7 &&
              run.elapsed_ticks() == 7 * pipeline::period && run.timer_clk2_edges() == 3,
          "FM A.2.3 arithmetic/transfer interlock retires its producer but holds E1/AG2/F2 for one "
          "real clock");
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  require(!stages(e1).memory_source_stall && stages(e1).memory &&
              machine.cpu().state().r[0] == pointer + 1 &&
              machine.bus().read({.space = DspSpace::x, .address = address}) ==
                  (transfer ? word_x : first_high) &&
              machine.bus().read({.space = DspSpace::y, .address = address}) ==
                  (transfer ? word_y : 0),
          "The following E1 reads the now-ready actual accumulator and the held younger AG2 "
          "publishes once");
  clocks(run, machine, 2);
  require(machine.cpu().state().pc == body + 3 && machine.cpu().state().a == value &&
              machine.cpu().state().r[0] == pointer + 1 && run.completed_cycles() == 10,
          "One stall clock is elapsed separately from the three actual native retirements");
}

struct DataFields {
  unsigned x = 0;
  unsigned y = 0;
  bool load_x = false;
  bool load_y = false;
};

void one_field_pair(const DataFields sample) {
  DspMachine machine;
  const auto x = "x" + std::to_string(sample.x);
  const auto y = "y" + std::to_string(sample.y);
  auto instruction = std::string("move ");
  instruction.append(sample.load_x ? "x:(r0)" : x);
  instruction.push_back(',');
  instruction.append(sample.load_x ? x : "x:(r0)");
  instruction.push_back(' ');
  instruction.append(sample.load_y ? "y:(r4)" : y);
  instruction.push_back(',');
  instruction.append(sample.load_y ? y : "y:(r4)");
  const std::array code{instruction};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = word_x});
  machine.bus().write({.space = DspSpace::y, .address = initial, .value = word_y});
  auto expected = machine.cpu().state();
  if (sample.load_x) {
    expected.x = sample.x == 0 ? pipeline::unit + word_x
                               : (pipeline::unit * word_x) + memory_pipeline::first;
  }
  if (sample.load_y) {
    expected.y = sample.y == 0 ? word_y : (pipeline::unit * word_y) + memory_pipeline::half;
  }
  ++expected.pc;
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 7);
  const auto x_store = sample.x == 0 ? memory_pipeline::first : 1U;
  const auto y_store = sample.y == 0 ? memory_pipeline::half : 0U;
  require(machine.cpu().state() == expected &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) ==
                  (sample.load_x ? word_x : x_store) &&
              machine.bus().read({.space = DspSpace::y, .address = initial}) ==
                  (sample.load_y ? word_y : y_store),
          "All16 dual data-field/direction cases retain literal sibling values and genuine zero "
          "stores");
}

void data_fields() {
  for (const unsigned x : {0U, 1U}) {
    for (const unsigned y : {0U, 1U}) {
      for (const bool load_x : {false, true}) {
        for (const bool load_y : {false, true}) {
          one_field_pair({.x = x, .y = y, .load_x = load_x, .load_y = load_y});
        }
      }
    }
  }
}

void same_instruction_store_reads_old_accumulator() {
  DspMachine machine;
  const std::array<std::string, 2> code{"add x1,a a10,l:(r0)+", "move a10,l:(r0)+"};
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  require(machine.cpu().state().a == 0 &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) == 0 &&
              machine.bus().read({.space = DspSpace::y, .address = initial}) == 0,
          "A parallel raw store reads its own instruction's old accumulator before the actual ALU "
          "result");
  const auto bubble = run.advance_cycle(machine, fetch_clock::input());
  require(stages(bubble).memory_source_stall && machine.cpu().state().a == pipeline::unit,
          "Only the next instruction waits for the actual preceding ALU accumulator write");
  clocks(run, machine, 2);
  require(machine.cpu().state().pc == body + 2 && run.completed_cycles() == 9 &&
              machine.bus().read({.space = DspSpace::x, .address = 0}) == 1 &&
              machine.bus().read({.space = DspSpace::y, .address = 0}) == 0,
          "The dependent later store samples the new native accumulator after one elapsed bubble");
}

void long_move_is_not_an_alu_producer() {
  DspMachine machine;
  const std::array<std::string, 3> code{"add x1,b l:(r0)+,a10", "tfr a,b", "move b10,l:(r0)+"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = word_x});
  machine.bus().write({.space = DspSpace::y, .address = initial, .value = word_y});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  require(machine.cpu().state().a == pair && machine.cpu().state().b == pipeline::unit,
          "A parallel long load publishes its actual A fields before the unrelated B ALU result");
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = stages(cycle);
  require(
      phase.e1_deferred_accumulators == 0 && phase.e1_operands && phase.candidate_inputs &&
          phase.e1_operands->a == pair && phase.candidate_inputs->a == pair &&
          machine.cpu().state().b == 2 * pipeline::unit,
      "Only preceding ALU writes defer adder operands; a real E1 long load is already available");
  clocks(run, machine, 3);
  require(
      machine.cpu().state().pc == body + 3 && machine.cpu().state().a == pair &&
          machine.cpu().state().b == pair && run.completed_cycles() == 10 &&
          machine.bus().read({.space = DspSpace::x, .address = 0}) == word_x &&
          machine.bus().read({.space = DspSpace::y, .address = 0}) == word_y,
      "The later B10 store has exactly its own native source bubble and preserves both raw words");
}

void assembler_preserves_requested_moves() {
  struct Encoding {
    const char *source;
    unsigned word;
  };

  // Independently encoded by the pinned NXP6.3.28 assembler; these are our own instructions.
  constexpr std::array expected{
      Encoding{.source = "mpy y0,x0,a x:(r0)+,x0 y:(r4)+,y0", .word = 0xf098d0},
      Encoding{.source = "mac y0,x0,a x:(r0)+,x1 y:(r4)+,y1", .word = 0xf598d2},
      Encoding{.source = "add x1,a a10,l:(r0)+", .word = 0x405860},
      Encoding{.source = "move x0,x:(r0)+ y0,y:(r4)+", .word = 0xb01800}};
  const dsp56k::Assembler assembler;
  for (const auto sample : expected) {
    const auto result = assembler.assemble(sample.source);
    require(result.success() && result.wordCount == 1 && result.word[0] == sample.word,
            "Native assembler must preserve every field of the independent own NXP encoding");
  }
  for (const auto *source : {"mpy y0,x0,a garbage",
                             "move x:(r0),x0 garbage",
                             "mpy y0,x0,a x:(r0)-n0,x0 y:(r4),y0",
                             "move x:(r0),x0 y:(r0),y0"}) {
    require(!assembler.assemble(source).success(),
            "Unsupported trailing fields and impossible dual AGU modes must fail instead of "
            "disappearing");
  }
}

void unrelated_store_does_not_stall() {
  DspMachine machine;
  const std::array<std::string, 2> code{"mpy y0,x0,a", "move b10,l:(r0)+"};
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  require(!stages(cycle).memory_source_stall && stages(cycle).memory &&
              stages(cycle).retirement_published &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) == 1 &&
              machine.bus().read({.space = DspSpace::y, .address = initial}) == 0,
          "An A producer does not invent an accumulator-source stall for a B10 store");
  clocks(run, machine, 1);
  require(machine.cpu().state().pc == body + 2 && run.completed_cycles() == 8,
          "The independent store retains normal filled-stage throughput");
}

}

void execution_checks() {
  assembler_preserves_requested_moves();
  native_parallel_alu();
  long_move_is_not_an_alu_producer();
  same_instruction_store_reads_old_accumulator();
  data_fields();
  unrelated_store_does_not_stall();
  for (const auto target : targets) { long_load(target); }
  for (const auto target : negative_targets) { long_load(target, negative_words); }
  raw_long_load_preserves_extension(false);
  raw_long_load_preserves_extension(true);
  long_store("a10", 0, 0);
  long_store("b10", 1, 0);
  long_store("x", 1, memory_pipeline::first);
  long_store("y", 0, memory_pipeline::half);
  accumulator_store_interlock(false);
  accumulator_store_interlock(true);
}
}
