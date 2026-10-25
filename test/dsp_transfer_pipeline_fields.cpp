// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_transfer_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::transfer_pipeline {
namespace {
inline constexpr unsigned short_data = 0x800000;
inline constexpr unsigned short_address = 0x80;
inline constexpr std::uint64_t product = 0x091a2b000000;

void independent_encodings() {
  struct Encoding {
    const char *source;
    std::array<unsigned, 2> words;
    unsigned count = 1;
  };

  // Our instructions independently encoded by the pinned NXP6.3.28 assembler.
  constexpr std::array inputs{
      Encoding{.source = "move #>$800001,x0", .words = {0x44f400, 0x800001}, .count = 2},
      Encoding{.source = "move #>$800001,a", .words = {0x56f400, 0x800001}, .count = 2},
      Encoding{.source = "move #>7,r0", .words = {0x60f400, 7}, .count = 2},
      Encoding{.source = "move #>7,n0", .words = {0x70f400, 7}, .count = 2},
      Encoding{.source = "move #>7,m0", .words = {0x05f420, 7}, .count = 2},
      Encoding{.source = "move #<7,r0", .words = {0x300700}},
      Encoding{.source = "move #<7,n0", .words = {0x380700}},
      Encoding{.source = "move #<7,m0", .words = {0x0507a0}},
      Encoding{.source = "move x1,r0", .words = {0x20b000}},
      Encoding{.source = "move x1,n0", .words = {0x20b800}},
      Encoding{.source = "move x1,m0", .words = {0x04c5a0}},
      Encoding{.source = "move x:(r0),m0", .words = {0x05e020}},
      Encoding{.source = "move y:(r0),m0", .words = {0x05e060}},
      Encoding{.source = "add x1,a x0,r0", .words = {0x209060}},
      Encoding{.source = "move #<$80,x0", .words = {0x248000}},
      Encoding{.source = "mpy y0,x0,a x1,r0", .words = {0x20b0d0}},
      Encoding{.source = "move x:(r0)+,r0", .words = {0x60d800}}};
  for (const auto input : inputs) {
    const auto actual = checkpoint::assemble(input.source);
    require(actual.size() == input.count, "Native assembler must preserve independent word count");
    for (unsigned index = 0; index < input.count; ++index) {
      require(actual.at(index) == input.words.at(index),
              std::string(input.source) +
                  ": native input must match the independent own NXP encoding");
    }
  }
}

struct Target {
  const char *name;
  DspMoveFields fields;
  std::uint64_t x = pipeline::unit + memory_pipeline::first;
  std::uint64_t y = memory_pipeline::half;
  std::uint64_t a = 0;
  std::uint64_t b = pipeline::unit;
};

constexpr std::array targets{Target{.name = "x0", .fields = {.x = 1}, .x = 0x1800001},
                             Target{.name = "x1", .fields = {.x = 2}, .x = 0x800001123456},
                             Target{.name = "y0", .fields = {.y = 1}, .y = sample},
                             Target{.name = "y1", .fields = {.y = 2}, .y = 0x800001400000},
                             Target{.name = "a0", .fields = {.a = 1}, .a = sample},
                             Target{.name = "a1", .fields = {.a = 2}, .a = 0x800001000000},
                             Target{.name = "a2", .fields = {.a = 4}, .a = 0x01000000000000},
                             Target{.name = "a", .fields = {.a = 7}, .a = 0xff800001000000},
                             Target{.name = "b0", .fields = {.b = 1}, .b = 0x1800001},
                             Target{.name = "b1", .fields = {.b = 2}, .b = 0x800001000000},
                             Target{.name = "b2", .fields = {.b = 4}, .b = 0x01000001000000},
                             Target{.name = "b", .fields = {.b = 7}, .b = 0xff800001000000}};

void literal_fields(const Target &target) {
  DspMachine machine;
  const std::array code{std::string("move #>$800001,") + target.name};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto ag1 = run.advance_cycle(machine, fetch_clock::input());
  require(stages(ag1).ag1_extension == DspProgramWord{.address = body + 1, .value = sample} &&
              machine.cpu().debug_state() == before,
          "AG1 retains the actual preceding F2 literal, without early destination/PC effects");
  clocks(run, machine, 1);
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = stages(e1);
  auto expected = before;
  expected.cpu.x = target.x;
  expected.cpu.y = target.y;
  expected.cpu.a = target.a;
  expected.cpu.b = target.b;
  require(
      phase.transfer && phase.transfer->fields == target.fields &&
          stages(e1).e1_extension == stages(ag1).ag1_extension &&
          machine.cpu().debug_state() == expected && e1.bus.core_access_count == 1 &&
          machine.reference_fetch().consume_address == body + 2,
      std::string(target.name) +
          ": native E1 consumes two captured words and publishes only actual destination fields");
  clocks(run, machine, 1);
  expected.cpu.pc += 2;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(machine.cpu().debug_state() == expected,
          "A two-word native transfer retires its true PC/cycle count without repeating E1");
}

void agu_destination(const char kind, const unsigned index, const bool short_word) {
  DspMachine machine;
  const auto reg = std::string(1, kind) + std::to_string(index);
  const std::array code{std::string(short_word ? "move #<7," : "move #>7,") + reg};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  auto expected = before;
  DspMoveFields fields;
  const auto bit = static_cast<std::uint8_t>(1U << index);
  if (kind == 'r') {
    expected.cpu.r.at(index) = 7;
    fields.r = bit;
  }
  if (kind == 'n') {
    expected.cpu.n.at(index) = 7;
    fields.n = bit;
  }
  if (kind == 'm') {
    expected.cpu.m.at(index) = 7;
    fields.m = bit;
  }
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  require(machine.cpu().debug_state() == before, "AGU immediate does not publish before E1");
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = stages(e1);
  require(phase.transfer && phase.transfer->fields == fields &&
              machine.cpu().debug_state() == expected &&
              stages(e1).e1_extension.has_value() != short_word,
          reg + ": actual native AGU register publication retains other CPU fields, even unchanged "
                "values");
  clocks(run, machine, 1);
  const auto words = short_word ? 1U : 2U;
  expected.cpu.pc += words;
  ++expected.native_instructions;
  expected.native_cycles += words;
  require(machine.cpu().debug_state() == expected,
          "Every R/N/M destination retains native short/long identity through E2");
}

void short_fraction() {
  DspMachine machine;
  const std::array<std::string, 2> code{"move #<$80,x0", "move #<$80,r0"};
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 8);
  require(machine.cpu().state().x == pipeline::unit + short_data &&
              machine.cpu().state().r[0] == short_address && machine.cpu().state().pc == body + 2 &&
              machine.cpu().state().sr == masked_status,
          "Actual short immediates retain fractional data alignment and integer AGU alignment");
}

void register_fields() {
  DspMachine machine;
  const std::array<std::string, 3> code{"move x0,y1", "move y1,n0", "move x1,m0"};
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 9);
  require(machine.cpu().state().y ==
                  (std::uint64_t{memory_pipeline::first} << word_bits) + memory_pipeline::half &&
              machine.cpu().state().n[0] == memory_pipeline::first &&
              machine.cpu().state().m[0] == 1 && machine.cpu().state().pc == body + 3 &&
              machine.cpu().state().sr == masked_status,
          "Register transfers sample live prior E1 XY fields and publish actual N/M destinations");
}

void parallel_register_transfer() {
  DspMachine machine;
  const std::array<std::string, 2> code{"mpy y0,x0,a x1,r0", "move x:(r0),y0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = other});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  require(machine.cpu().state().r[0] == 1 && machine.cpu().state().a == 0 &&
              machine.cpu().state().sr == masked_status,
          "Parallel native transfer publishes only actual R in E1, holding multiply/CCR for E2");
  clocks(run, machine, 5);
  require(machine.cpu().state().a == product && machine.cpu().state().r[0] == 1 &&
              machine.cpu().state().y == other && machine.cpu().state().pc == body + 2 &&
              machine.cpu().state().sr == (masked_status | unnormalized),
          "Actual old MPY operands and flags survive R publication and three decoded AGU holds");
}
}

void field_checks() {
  independent_encodings();
  for (const auto &target : targets) { literal_fields(target); }
  for (const char kind : {'r', 'n', 'm'}) {
    for (unsigned index = 0; index < 8; ++index) {
      agu_destination(kind, index, false);
      agu_destination(kind, index, true);
    }
  }
  short_fraction();
  register_fields();
  parallel_register_transfer();
}
}
