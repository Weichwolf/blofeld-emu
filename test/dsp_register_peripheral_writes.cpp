// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_pipeline_fixture.hpp"
#include "dsp_register_peripheral_encodings.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::test::peripheral_pipeline {
namespace {
struct RegisterCase {
  std::string source;
  unsigned index;
  unsigned value;
  bool boundary = false;
};

void one_register(const RegisterCase &sample) {
  DspMachine machine;
  const std::array code{"movep " + sample.source + ",x:<<$fffffc"};
  prepare(machine,
          code,
          {.index = sample.index,
           .pointer = sample.boundary ? sample.value : 7U,
           .offset = sample.boundary ? sample.value : 3U,
           .modifier = sample.boundary ? sample.value : 7U});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto ag1 = stages(run.advance_cycle(machine, fetch_clock::input()));
  require(ag1.ag1_address && ag1.ag1_address->kind == DspAddressStageKind::peripheral_register &&
              !ag1.ag1_extension && !ag1.ag1_address->extension &&
              !ag1.ag1_address->pointer_update && machine.cpu().debug_state() == before &&
              !machine.bus().once_latch(),
          "Register MOVEP captures its fixed port with no extension, AGU input or early write");
  const auto ag2 = stages(run.advance_cycle(machine, fetch_clock::input()));
  require(ag2.ag2_address && ag2.ag2_address->published && !ag2.ag2_address->pointer_update &&
              machine.cpu().debug_state() == before && !machine.bus().once_latch(),
          "Fixed-port AG2 preserves all R/N/M state and performs no I/O");
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  require(e1.bus.core_access_count == 2 && e1.bus.memory &&
              e1.bus.memory->data.kind == DspNativeAccessKind::write &&
              e1.bus.memory->data.target.space == DspSpace::x &&
              e1.bus.memory->data.target.address == once &&
              e1.bus.memory->data.value == sample.value && !e1.bus.memory->move &&
              machine.cpu().debug_state() == before && machine.bus().once_latch() == sample.value,
          "Native E1 samples the actual source and commits one port write with real P fetch");
  const auto e2 = stages(run.advance_cycle(machine, fetch_clock::input()));
  auto expected = before;
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(e2.retirement_published && machine.cpu().debug_state() == expected &&
              machine.bus().once_latch() == sample.value && run.completed_cycles() == 7,
          "E2 retires the independently specified one-cycle MOVEP without a repeated write");
}

void every_register() {
  for (const auto &sample : std::array<std::pair<const char *, unsigned>, 4>{
           {{"x0", memory_pipeline::first}, {"x1", 1}, {"y0", memory_pipeline::half}, {"y1", 0}}}) {
    one_register({.source = sample.first, .index = 0, .value = sample.second});
  }
  for (unsigned index = 0; index < 8; ++index) {
    const auto suffix = std::to_string(index);
    one_register({.source = "r" + suffix, .index = index, .value = 7});
    one_register({.source = "n" + suffix, .index = index, .value = 3});
    one_register({.source = "m" + suffix, .index = index, .value = 7});
  }
}

void register_boundaries() {
  constexpr std::array values{0U, 0x800001U, word_mask};
  for (const char group : {'r', 'n', 'm'}) {
    for (unsigned index = 0; index < 8; ++index) {
      const auto source = std::string(1, group) + std::to_string(index);
      for (const unsigned value : values) {
        one_register({.source = source, .index = index, .value = value, .boundary = true});
      }
    }
  }
}

void low_ports() {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    DspMachine machine;
    const unsigned value = space == DspSpace::x ? 9U : 0x100U;
    const unsigned address = space == DspSpace::x ? clock_control : 0xffff9fU;
    const std::string name = space == DspSpace::x ? "x" : "y";
    const std::array code{"movep r1," + name + ":<<" + number(address)};
    prepare(machine, code, {.index = 1, .pointer = value});
    auto expected = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 7);
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
    require(machine.cpu().debug_state() == expected &&
                machine.bus().read({.space = space, .address = address}) == value,
            "Low X/Y register MOVEP selects the actual strict owner without changing its source");
  }
}

void older_register_producer() {
  DspMachine machine;
  const std::array<std::string, 2> code{"move x1,r1", "movep r1,x:<<$fffffc"};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 8);
  expected.cpu.pc = body + 2;
  expected.cpu.r[1] = 1;
  expected.native_instructions += 2;
  expected.native_cycles += 2;
  require(machine.cpu().debug_state() == expected && machine.bus().once_latch() == 1,
          "Register MOVEP samples its predecessor's actual R1 publication in the same cycle");
}

void younger_same_register_update() {
  DspMachine machine;
  const std::array<std::string, 2> code{"movep r0,x:<<$fffffc", "move x:(r0)+,x0"};
  prepare(machine, code, {.modifier = memory_pipeline::linear_modifier});
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = literal});
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  const auto overlap = run.advance_cycle(machine, fetch_clock::input());
  require(overlap.bus.memory && overlap.bus.address_update &&
              overlap.bus.memory->inputs.r[0] == 7 && overlap.bus.memory->data.value == 7 &&
              overlap.bus.address_update->update.before == 7 &&
              overlap.bus.address_update->update.value == 8 && machine.cpu().state().r[0] == 8 &&
              machine.bus().once_latch() == 7,
          "Older native E1 writes old R0 while younger AG2 independently publishes incremented R0");
  clocks(run, machine, 2);
  expected.cpu.pc = body + 2;
  expected.cpu.r[0] = 8;
  expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | literal;
  expected.native_instructions += 2;
  expected.native_cycles += 2;
  require(machine.cpu().debug_state() == expected && machine.bus().once_latch() == 7 &&
              machine.bus().read({.space = DspSpace::x, .address = 7}) == literal,
          "Both retirements retain younger R0 and real data without a stall or extra port write");
}

struct Window {
  const char *source;
  unsigned value;
  bool alias;
  unsigned clocks;
};

std::optional<unsigned> retirement(const DspFetchCycleResult &cycle) {
  if (!cycle.pipeline || !cycle.pipeline->retirement_published) { return std::nullopt; }
  const auto word = cycle.pipeline->execute2;
  if (!word) { throw Fault("Native register MOVEP window lost its real E2 word"); }
  return word->address;
}

void independent_window(const Window sample) {
  constexpr unsigned quiet = 8;
  constexpr unsigned window_instructions = 17;
  constexpr unsigned window_cycles = 19;
  constexpr unsigned canary = 0x100;
  constexpr unsigned x_canary = 0x13579b;
  constexpr unsigned y_canary = 0x2468ac;
  const bool immediate = std::string_view(sample.source) == "immediate";
  const unsigned write_pc = body + 4 + quiet;
  const unsigned after_pc = write_pc + (immediate ? 2U : 1U) + (sample.alias ? 1U : 0U) + 5;
  std::vector<std::string> code{"move #>" + number(sample.value) + ",r1",
                                "move #>" + number(sample.value) + ",x0"};
  code.insert(code.end(), quiet, "nop");
  const auto source = immediate ? "#>" + number(sample.value) : std::string(sample.source);
  code.push_back("movep " + source + ",x:<<$ffff90");
  if (sample.alias) { code.emplace_back("move x:(r0)+,x0"); }
  code.insert(code.end(), quiet, "nop");
  DspMachine machine;
  prepare(machine, code, {.pointer = sample.value, .modifier = memory_pipeline::linear_modifier});
  machine.bus().write({.space = DspSpace::x, .address = sample.value, .value = literal});
  machine.bus().write({.space = DspSpace::x, .address = canary, .value = x_canary});
  machine.bus().write({.space = DspSpace::y, .address = canary, .value = y_canary});
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  std::optional<std::uint64_t> before;
  std::optional<std::uint64_t> after;
  for (unsigned clock = 0; clock < 64 && !after; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto pc = retirement(cycle);
    if (pc == write_pc - 1) { before = cycle.completed_cycles; }
    if (pc == after_pc) { after = cycle.completed_cycles; }
  }
  require(
      before && after && *after - *before == sample.clocks,
      "Register/immediate/adjacent-AGU quiet window matches the independent primary/model clocks");
  expected.cpu.pc = after_pc + 1;
  expected.cpu.r[0] = sample.value + (sample.alias ? 1U : 0U);
  expected.cpu.r[1] = sample.value;
  expected.cpu.x =
      (expected.cpu.x & ~std::uint64_t{word_mask}) | (sample.alias ? literal : sample.value);
  expected.native_instructions += window_instructions + (sample.alias ? 1U : 0U);
  expected.native_cycles += window_cycles + (sample.alias ? 1U : 0U) + (immediate ? 1U : 0U);
  require(machine.cpu().debug_state() == expected &&
              machine.bus().shi().read(clock_control) == sample.value &&
              machine.bus().read({.space = DspSpace::x, .address = canary}) == x_canary &&
              machine.bus().read({.space = DspSpace::y, .address = canary}) == y_canary &&
              machine.bus().read({.space = DspSpace::x, .address = sample.value}) == literal,
          "Complete quiet-window register/CCR/counter/I/O state and RAM canaries match");
}
}

void register_writes() {
  constexpr unsigned r0_alias_value = 17;
  constexpr unsigned r1_alias_value = 19;
  register_encodings();
  every_register();
  register_boundaries();
  low_ports();
  older_register_producer();
  younger_same_register_update();
  for (const auto sample :
       {Window{.source = "x0", .value = 3, .alias = false, .clocks = 7},
        Window{.source = "r1", .value = 9, .alias = false, .clocks = 7},
        Window{.source = "r0", .value = 11, .alias = false, .clocks = 7},
        Window{.source = "immediate", .value = 13, .alias = false, .clocks = 8},
        Window{.source = "r0", .value = r0_alias_value, .alias = true, .clocks = 8},
        Window{.source = "r1", .value = r1_alias_value, .alias = true, .clocks = 8}}) {
    independent_window(sample);
  }
}
}
