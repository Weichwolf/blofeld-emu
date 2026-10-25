// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_writeback_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::writeback;
namespace access = blofeld::test::reference;
using native_access::fault;
constexpr unsigned seed_steps = 11;

struct DataTarget {
  const char *name;
  DspMoveFields fields;
  std::uint64_t x = seed_x, y = seed_y, a = seed_a, b = seed_b;
};

constexpr std::array targets{DataTarget{.name = "x0", .fields = {.x = 1}, .x = 0x112233800001},
                             DataTarget{.name = "x1", .fields = {.x = 2}, .x = 0x800001445566},
                             DataTarget{.name = "y0", .fields = {.y = 1}, .y = 0x223344800001},
                             DataTarget{.name = "y1", .fields = {.y = 2}, .y = 0x800001556677},
                             DataTarget{.name = "a0", .fields = {.a = 1}, .a = 0x12345678800001},
                             DataTarget{.name = "a1", .fields = {.a = 2}, .a = 0x128000019abcde},
                             DataTarget{.name = "a2", .fields = {.a = 4}, .a = 0x013456789abcde},
                             DataTarget{.name = "a", .fields = {.a = 7}, .a = 0xff800001000000},
                             DataTarget{.name = "b0", .fields = {.b = 1}, .b = 0x23456789800001},
                             DataTarget{.name = "b1", .fields = {.b = 2}, .b = 0x23800001abcdef},
                             DataTarget{.name = "b2", .fields = {.b = 4}, .b = 0x01456789abcdef},
                             DataTarget{.name = "b", .fields = {.b = 7}, .b = 0xff800001000000}};

void data_load(const DataTarget &target, const DspSpace space) {
  DspMachine machine;
  DspProgram code;
  seed(code);
  const auto body = code.pc();
  const std::string area = space == DspSpace::x ? "x" : "y";
  code.emit("move " + area + ":>$100," + target.name);
  code.install(machine.bus());
  machine.bus().write({.space = space, .address = source, .value = sample});
  checkpoint::run(machine, seed_steps);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.x == seed_x && before.cpu.y == seed_y && before.cpu.a == seed_a &&
              before.cpu.b == seed_b,
          "Own literal full-register seeds are established by actual native instructions");
  auto expected = before;
  expected.cpu.x = target.x;
  expected.cpu.y = target.y;
  expected.cpu.a = target.a;
  expected.cpu.b = target.b;
  auto progress = await_move(machine, machine.begin_reference_instruction(execution));
  const auto held = move(progress);
  require(held.fields == target.fields && held.pc == body && machine.cpu().debug_state() == before,
          std::string(target.name) +
              ": actual load completes without publishing register/flags early");
  const auto saved = machine.checkpoint();
  const auto stale = access::ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(stale)); }));
    machine.bus().write({.space = space, .address = source, .value = access::third});
    progress = machine.advance_reference_access(access::ticket(machine));
    const auto actual = machine.cpu().debug_state();
    require(actual == expected,
            std::string(target.name) + ": E1 publishes the independently expected word slice only" +
                " actual A=" + number(actual.cpu.a) + " B=" + number(actual.cpu.b) +
                " SR=" + number(actual.cpu.sr) + " expected A=" + number(expected.cpu.a) +
                " B=" + number(expected.cpu.b) + " SR=" + number(expected.cpu.sr));
    const auto after_e1 = machine.checkpoint();
    const auto retired_ticket = access::ticket(progress);
    DspMachine fork;
    fork.restore(after_e1);
    static_cast<void>(
        fault([&] { static_cast<void>(fork.advance_reference_access(retired_ticket)); }));
    const auto retired = retire(fork, {.pending = fork.reference_pending()});
    const auto after = fork.cpu().debug_state();
    expected.cpu.pc = body + 2;
    require(retired.retired && after.cpu == expected.cpu &&
                after.native_instructions == before.native_instructions + 1,
            std::string(target.name) +
                ": a fresh post-E1 fork retains the captured value and retires once");
    expected.cpu.pc = before.cpu.pc;
  }
}

void agu_load(const char kind, const unsigned index) {
  DspMachine machine;
  DspProgram code;
  seed(code);
  const auto body = code.pc();
  const auto name = std::string(1, kind) + std::to_string(index);
  code.emit("move x:>$100," + name);
  code.install(machine.bus());
  constexpr unsigned modifier_value = 7;
  const auto value = kind == 'm' ? modifier_value : sample;
  machine.bus().write({.space = DspSpace::x, .address = source, .value = value});
  checkpoint::run(machine, seed_steps);
  const auto before = machine.cpu().debug_state();
  auto expected = before;
  DspMoveFields fields;
  const auto bit = static_cast<std::uint8_t>(1U << index);
  if (kind == 'r') {
    fields.r = bit;
    expected.cpu.r.at(index) = value;
  }
  if (kind == 'n') {
    fields.n = bit;
    expected.cpu.n.at(index) = value;
  }
  if (kind == 'm') {
    fields.m = bit;
    expected.cpu.m.at(index) = value;
  }
  auto progress = await_move(machine, machine.begin_reference_instruction(execution));
  require(move(progress).fields == fields && machine.cpu().debug_state() == before,
          name + ": a real control load has one declared register destination");
  progress = machine.advance_reference_access(access::ticket(progress));
  require(machine.cpu().debug_state() == expected,
          name + ": E1 retains every unrelated register, flags, PC and native counter");
  expected.cpu.pc = body + 2;
  require(retire(machine, progress).retired && machine.cpu().state() == expected.cpu,
          name + ": actual captured R/N/M result retires without a second load");
}

struct LongTarget {
  const char *name;
  DspMoveFields fields;
  std::uint64_t x = seed_x, y = seed_y, a = seed_a, b = seed_b;
};

constexpr std::array long_targets{
    LongTarget{.name = "a10", .fields = {.a = 3}, .a = 0x12234567345678},
    LongTarget{.name = "b10", .fields = {.b = 3}, .b = 0x23234567345678},
    LongTarget{.name = "x", .fields = {.x = 3}, .x = 0x234567345678},
    LongTarget{.name = "y", .fields = {.y = 3}, .y = 0x234567345678},
    LongTarget{.name = "a", .fields = {.a = 7}, .a = 0x00234567345678},
    LongTarget{.name = "b", .fields = {.b = 7}, .b = 0x00234567345678},
    LongTarget{
        .name = "ab", .fields = {.a = 7, .b = 7}, .a = 0x00234567000000, .b = 0x00345678000000},
    LongTarget{
        .name = "ba", .fields = {.a = 7, .b = 7}, .a = 0x00345678000000, .b = 0x00234567000000}};

void long_load(const LongTarget &target, const unsigned encoding) {
  DspMachine machine;
  DspProgram code;
  seed(code);
  // FM MOVE long: 0100L0LLW1MMMRRR00000000; W=1, MMM=6, RRR=0.
  constexpr unsigned base = 0x40f000;
  constexpr unsigned high_shift = 19;
  constexpr unsigned low_shift = 16;
  code.word(base | ((encoding >> 2U) << high_shift) | ((encoding & 3U) << low_shift));
  code.word(source);
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = source, .value = access::second});
  machine.bus().write({.space = DspSpace::y, .address = source, .value = access::third});
  checkpoint::run(machine, seed_steps);
  const auto before = machine.cpu().debug_state();
  auto expected = before;
  expected.cpu.x = target.x;
  expected.cpu.y = target.y;
  expected.cpu.a = target.a;
  expected.cpu.b = target.b;
  auto progress = await_move(machine, machine.begin_reference_instruction(execution));
  require(move(progress).fields == target.fields && machine.cpu().debug_state() == before,
          std::string(target.name) + ": both genuine long operands precede the single E1 group");
  progress = machine.advance_reference_access(access::ticket(progress));
  require(machine.cpu().debug_state() == expected,
          std::string(target.name) + ": literal long destination fields publish atomically");
  require(retire(machine, progress).retired, "The actual long move retires after E1");
}
}

int main() {
  try {
    for (const auto &target : targets) {
      for (const auto space : {DspSpace::x, DspSpace::y}) { data_load(target, space); }
    }
    for (const char kind : {'r', 'n', 'm'}) {
      for (unsigned index = 0; index < 8; ++index) { agu_load(kind, index); }
    }
    for (unsigned index = 0; index < long_targets.size(); ++index) {
      long_load(long_targets.at(index), index);
    }
    parallel_checks();
    clock_checks();
    std::cout << "Native move writeback:56 fixed load destinations and phase checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
