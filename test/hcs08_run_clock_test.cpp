// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_wire;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr auto timing_contract = Hcs08RunContract::reference_instruction_retirement;
constexpr unsigned seed_a = 0xa5, result_a = 0x55, initial_flags = 0x79;
constexpr unsigned expected_hx = 0x1278, expected_sp = 0x1277, sentinel = 0xaa;
constexpr unsigned store = 0x80, period_count = 16, held_periods = 7;
constexpr std::array<std::uint8_t, 10> own_code{
    0xae, 0x78, 0x94, 0xa6, result_a, 0xc7, 0, store, 0x9d, 0x82};

void install(Target &target) {
  target.memory.install(algorithm, own_code);
  target.memory.bytes[store] = sentinel;
  target.cpu.set_state(
      {.pc = algorithm, .hx = seed, .sp = stack, .a = seed_a, .ccr = initial_flags});
}

void go(Hcs08Bdc &bdc) {
  constexpr std::array<std::uint8_t, 1> frame{0x08};
  blofeld::test::hcs08_bdc::command(bdc, frame);
}

void expect(const Target &target, const Hcs08Bdc &bdc, const unsigned period) {
  constexpr std::array<unsigned, 6> boundaries{2, 4, 6, 10, 11, 16};
  constexpr std::array<unsigned, 6> pcs{0xa2, 0xa3, 0xa5, 0xa8, 0xa9, 0xaa};
  unsigned retired = 0;
  for (const auto end : boundaries) { retired += static_cast<unsigned>(period >= end); }
  const Hcs08State expected{
      .pc = static_cast<std::uint16_t>(retired == 0 ? algorithm : pcs[retired - 1]),
      .hx = static_cast<std::uint16_t>(period < 2 ? seed : expected_hx),
      .sp = static_cast<std::uint16_t>(period < 4 ? stack : expected_sp),
      .a = static_cast<std::uint8_t>(period < 6 ? seed_a : result_a),
      .ccr = initial_flags};
  require(
      target.cpu.state() == expected && target.memory.retired == retired &&
          target.memory.bytes[store] == (period < 10 ? sentinel : result_a) &&
          bdc.state().active == (period >= period_count) &&
          target.cpu.background_requested() == (period >= period_count),
      "Literal documented costs and independent register/store/BGND expectations at every period");
}

void advance(Target &target,
             Hcs08Bdc &bdc,
             Hcs08RunClock &clock,
             const unsigned first,
             std::ostream &trace) {
  for (unsigned period = first; period <= period_count; ++period) {
    clock.advance();
    expect(target, bdc, period);
    const auto cpu = target.cpu.state();
    trace << period << ' ' << cpu.pc << ' ' << static_cast<unsigned>(cpu.a) << ' ' << cpu.hx << ' '
          << cpu.sp << ' ' << static_cast<unsigned>(cpu.ccr) << ' '
          << static_cast<unsigned>(target.memory.bytes[store]) << ' '
          << clock.state().remaining_periods << '\n';
  }
}

void costs_and_replay() {
  Target target;
  install(target);
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08RunClock clock(target.memory, bdc, timing_contract);
  for (unsigned n = 0; n < held_periods; ++n) { clock.advance(); }
  require(clock.state().offered_periods == held_periods && clock.state().executing_periods == 0 &&
              target.cpu.state().pc == algorithm && target.memory.retired == 0,
          "Active BDC offers real clock periods without executing or fetching native instructions");
  go(bdc);
  for (unsigned period = 1; period <= 5; ++period) {
    clock.advance();
    expect(target, bdc, period);
  }
  const auto memory = target.memory;
  const auto cpu = target.cpu.checkpoint();
  const auto controller = bdc.checkpoint();
  const auto timer = clock.checkpoint();
  require(clock.state().remaining_periods == 1,
          "Checkpoint preserves an actual partial LDA budget");
  std::ostringstream expected;
  advance(target, bdc, clock, 6, expected);
  for (unsigned replay = 0; replay < 2; ++replay) {
    Target restored(&memory);
    restored.cpu.restore_checkpoint(cpu);
    Hcs08Bdc other(restored.cpu, restored.mapping, chip, Hcs08BdcReset::normal);
    other.restore_checkpoint(controller);
    Hcs08RunClock fresh(restored.memory, other, timing_contract);
    fresh.restore_checkpoint(timer);
    std::ostringstream actual;
    advance(restored, other, fresh, 6, actual);
    require(actual.str() == expected.str() && fresh.state() == clock.state() &&
                restored.memory.bytes == target.memory.bytes && other.state() == bdc.state(),
            "Fresh native CPU/RAM/BDC/clock owners reproduce every partial-instruction suffix");
  }
  constexpr unsigned finished_periods = held_periods + period_count;
  for (unsigned n = 0; n < 5; ++n) { clock.advance(); }
  require(clock.state().offered_periods == finished_periods + 5 &&
              clock.state().executing_periods == period_count &&
              clock.state().retired_instructions == 6 && !clock.faulted(),
          "BGND's five periods are followed by separate indefinite held time, without speculative "
          "fetch");
}

// Own architectural program: ordinary stack RAM, independent of the original
// programmer fixture's deliberately restricted data-write footprint.
class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(const std::uint16_t address, const ExecutionContext context) override {
    throw Fault("Unexpected own-program BDC read address=" + hex(address) +
                " pc=" + hex(context.pc));
  }

  void write(const ByteWrite access, const ExecutionContext context) override {
    throw Fault("Unexpected own-program BDC write address=" + hex(access.address) +
                " pc=" + hex(context.pc));
  }
};

struct MultiplyTarget {
  blofeld::test::hcs08::Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  NoBdcMemory mapping;

  explicit MultiplyTarget(const blofeld::test::hcs08::Memory *snapshot = nullptr)
      : memory(snapshot != nullptr ? *snapshot : blofeld::test::hcs08::Memory{}) {
    memory.writable.set(stack);
    memory.writable.set(stack - 1);
    cpu.set_state({.pc = algorithm, .hx = seed, .sp = stack, .a = seed_a, .ccr = initial_flags});
  }
};

void expect_multiply(const MultiplyTarget &target, const Hcs08Bdc &bdc, const unsigned period) {
  constexpr std::array<unsigned, 10> boundaries{2, 4, 9, 11, 12, 15, 21, 22, 28, 33};

  struct Registers {
    unsigned pc, hx, sp, a, ccr;
  };

  // Literal instruction expectations, independent of the timing decoder:
  // LDA, LDX, MUL17*19=0143 (H/C cleared), PSHA, CLRA, PULA, JSR, TAX, RTS, BGND.
  constexpr std::array<Registers, 11> states{
      {{.pc = 0xa0, .hx = 0x1200, .sp = 0x47c, .a = 0xa5, .ccr = 0x79},
       {.pc = 0xa2, .hx = 0x1200, .sp = 0x47c, .a = 0x11, .ccr = 0x79},
       {.pc = 0xa4, .hx = 0x1213, .sp = 0x47c, .a = 0x11, .ccr = 0x79},
       {.pc = 0xa5, .hx = 0x1201, .sp = 0x47c, .a = 0x43, .ccr = 0x68},
       {.pc = 0xa6, .hx = 0x1201, .sp = 0x47b, .a = 0x43, .ccr = 0x68},
       {.pc = 0xa7, .hx = 0x1201, .sp = 0x47b, .a = 0, .ccr = 0x6a},
       {.pc = 0xa8, .hx = 0x1201, .sp = 0x47c, .a = 0x43, .ccr = 0x6a},
       {.pc = 0xc0, .hx = 0x1201, .sp = 0x47a, .a = 0x43, .ccr = 0x6a},
       {.pc = 0xc1, .hx = 0x1243, .sp = 0x47a, .a = 0x43, .ccr = 0x6a},
       {.pc = 0xab, .hx = 0x1243, .sp = 0x47c, .a = 0x43, .ccr = 0x6a},
       {.pc = 0xac, .hx = 0x1243, .sp = 0x47c, .a = 0x43, .ccr = 0x6a}}};
  unsigned retired = 0;
  for (const auto end : boundaries) { retired += static_cast<unsigned>(period >= end); }
  const auto row = states.at(retired);
  const Hcs08State expected{.pc = static_cast<std::uint16_t>(row.pc),
                            .hx = static_cast<std::uint16_t>(row.hx),
                            .sp = static_cast<std::uint16_t>(row.sp),
                            .a = static_cast<std::uint8_t>(row.a),
                            .ccr = static_cast<std::uint8_t>(row.ccr)};
  require(target.cpu.state() == expected && target.memory.retired == retired &&
              bdc.state().active == (period >= boundaries.back()),
          "Independent multiply/stack/call state at every period " + std::to_string(period));
  constexpr unsigned call_period = 21;
  constexpr unsigned return_low = 0xab;
  constexpr unsigned product_low = 0x43;
  if (period >= 11) {
    require(target.memory.bytes[stack] == (period < call_period ? product_low : return_low),
            "PSHA/JSR write the independently expected stack byte only at retirement");
  }
  if (period >= call_period) {
    require(target.memory.known[stack - 1] && target.memory.bytes[stack - 1] == 0,
            "JSR writes both return bytes, with the documented stack order");
  }
}

void multiply_and_call() {
  MultiplyTarget target;
  constexpr unsigned subroutine = 0xc0;
  constexpr unsigned final_period = 33;
  constexpr unsigned cut_period = 6;
  constexpr std::array<std::uint8_t, 12> code{
      0xa6, 17, 0xae, 19, 0x42, 0x87, 0x4f, 0x86, 0xcd, 0, subroutine, 0x82};
  constexpr std::array<std::uint8_t, 2> callee{0x97, 0x81};
  target.memory.install(algorithm, code);
  target.memory.install(subroutine, callee);
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08RunClock clock(target.memory, bdc, timing_contract);
  go(bdc);
  for (unsigned period = 1; period <= cut_period; ++period) {
    clock.advance();
    expect_multiply(target, bdc, period);
  }
  const auto memory = target.memory;
  const auto cpu = target.cpu.checkpoint();
  const auto controller = bdc.checkpoint();
  const auto timer = clock.checkpoint();
  require(clock.state().remaining_periods == 3 && clock.state().retired_instructions == 2,
          "Joint snapshot is inside the five-period native MUL before its register effects");
  for (unsigned period = cut_period + 1; period <= final_period; ++period) {
    clock.advance();
    expect_multiply(target, bdc, period);
  }
  for (unsigned replay = 0; replay < 2; ++replay) {
    MultiplyTarget restored(&memory);
    restored.cpu.restore_checkpoint(cpu);
    Hcs08Bdc other(restored.cpu, restored.mapping, chip, Hcs08BdcReset::normal);
    other.restore_checkpoint(controller);
    Hcs08RunClock fresh(restored.memory, other, timing_contract);
    fresh.restore_checkpoint(timer);
    for (unsigned period = cut_period + 1; period <= final_period; ++period) {
      fresh.advance();
      expect_multiply(restored, other, period);
    }
    require(fresh.state() == clock.state() && other.state() == bdc.state() &&
                restored.memory.bytes == target.memory.bytes &&
                restored.memory.known == target.memory.known &&
                restored.memory.accesses == target.memory.accesses,
            "Fresh CPU/RAM/BDC/clock restoration preserves the complete MUL/call suffix");
  }
}

void decoder_and_unknowns() {
  using blofeld::test::hcs08::Memory;
  constexpr unsigned entry = 0x8000;
  constexpr unsigned flags = 0x68;

  struct Expected {
    unsigned opcode, bytes, periods;
  };

  constexpr std::array<Expected, 50> documented{
      {{.opcode = 0x0e, .bytes = 3, .periods = 5}, {.opcode = 0x22, .bytes = 2, .periods = 3},
       {.opcode = 0x23, .bytes = 2, .periods = 3}, {.opcode = 0x26, .bytes = 2, .periods = 3},
       {.opcode = 0xcc, .bytes = 3, .periods = 4}, {.opcode = 0x87, .bytes = 1, .periods = 2},
       {.opcode = 0x89, .bytes = 1, .periods = 2}, {.opcode = 0x8b, .bytes = 1, .periods = 2},
       {.opcode = 0x86, .bytes = 1, .periods = 3}, {.opcode = 0x8a, .bytes = 1, .periods = 3},
       {.opcode = 0x97, .bytes = 1, .periods = 1}, {.opcode = 0x9f, .bytes = 1, .periods = 1},
       {.opcode = 0x4f, .bytes = 1, .periods = 1}, {.opcode = 0x4c, .bytes = 1, .periods = 1},
       {.opcode = 0xe6, .bytes = 2, .periods = 3}, {.opcode = 0xee, .bytes = 2, .periods = 3},
       {.opcode = 0xeb, .bytes = 2, .periods = 3}, {.opcode = 0xe7, .bytes = 2, .periods = 3},
       {.opcode = 0xf1, .bytes = 1, .periods = 3}, {.opcode = 0xf9, .bytes = 1, .periods = 3},
       {.opcode = 0xf7, .bytes = 1, .periods = 2}, {.opcode = 0xfc, .bytes = 1, .periods = 3},
       {.opcode = 0xa4, .bytes = 2, .periods = 2}, {.opcode = 0xae, .bytes = 2, .periods = 2},
       {.opcode = 0xa6, .bytes = 2, .periods = 2}, {.opcode = 0xc7, .bytes = 3, .periods = 4},
       {.opcode = 0x94, .bytes = 1, .periods = 2}, {.opcode = 0x9d, .bytes = 1, .periods = 1},
       {.opcode = 0x82, .bytes = 1, .periods = 5}, {.opcode = 0x45, .bytes = 3, .periods = 3},
       {.opcode = 0x8c, .bytes = 1, .periods = 1}, {.opcode = 0x5f, .bytes = 1, .periods = 1},
       {.opcode = 0x20, .bytes = 2, .periods = 3}, {.opcode = 0x7f, .bytes = 1, .periods = 4},
       {.opcode = 0xaf, .bytes = 2, .periods = 2}, {.opcode = 0x65, .bytes = 3, .periods = 3},
       {.opcode = 0x25, .bytes = 2, .periods = 3}, {.opcode = 0x35, .bytes = 2, .periods = 4},
       {.opcode = 0xb7, .bytes = 2, .periods = 3}, {.opcode = 0x6e, .bytes = 3, .periods = 4},
       {.opcode = 0x3f, .bytes = 2, .periods = 5}, {.opcode = 0xb6, .bytes = 2, .periods = 3},
       {.opcode = 0xa5, .bytes = 2, .periods = 2}, {.opcode = 0x27, .bytes = 2, .periods = 3},
       {.opcode = 0xc6, .bytes = 3, .periods = 4}, {.opcode = 0xa1, .bytes = 2, .periods = 2},
       {.opcode = 0xcd, .bytes = 3, .periods = 6}, {.opcode = 0xa7, .bytes = 2, .periods = 2},
       {.opcode = 0x9e, .bytes = 3, .periods = 5}, {.opcode = 0x55, .bytes = 2, .periods = 4}}};
  Memory memory;
  for (const auto expected : documented) {
    constexpr std::uint8_t stack_load = 0xfe;
    const std::array<std::uint8_t, 4> code{
        static_cast<std::uint8_t>(expected.opcode), stack_load, store, 0};
    memory.install(entry, code);
    require(hcs08_instruction_timing(memory, {.pc = entry, .ccr = flags}) ==
                Hcs08InstructionTiming{.bytes = expected.bytes, .periods = expected.periods},
            "All fifty previously qualified first-opcode timing tuples remain exact");
  }
  constexpr std::array<std::uint8_t, 2> immediate{0xae, 0x78};
  memory.install(entry, immediate);
  memory.known.reset(entry + 1);
  require(rejected([&] {
            static_cast<void>(hcs08_instruction_timing(memory, {.pc = entry}));
          }).contains("fetch address=0x8001"),
          "An unknown immediate byte cannot be hidden by opcode-only timing");
}

void quarantine() {
  for (unsigned failure = 0; failure < 3; ++failure) {
    Target target;
    install(target);
    Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
    Hcs08RunClock clock(target.memory, bdc, timing_contract);
    go(bdc);
    clock.advance();
    const auto before = target.cpu.state();
    if (failure == 0) {
      constexpr std::array<std::uint8_t, 1> changed{0x56};
      target.memory.install(algorithm + 1, changed);
    } else if (failure == 1) {
      constexpr std::array<std::uint8_t, 1> background{0x90};
      blofeld::test::hcs08_bdc::command(bdc, background);
    } else {
      auto changed = before;
      changed.a = 0;
      target.cpu.set_state(changed);
    }
    const auto reason = rejected([&] { clock.advance(); });
    require(reason.contains(failure == 1 ? "BDC halt" : "changed during") && clock.faulted() &&
                clock.state().retired_instructions == 0 &&
                rejected([&] { clock.advance(); }).contains("coordinated recovery") &&
                rejected([&] {
                  static_cast<void>(clock.checkpoint());
                }).contains("coordinated recovery"),
            "Mid-instruction code/register mutation or halt cannot commit a stale budget or retry");
  }
  Target target;
  constexpr std::array<std::uint8_t, 5> unknown_store{0xa6, result_a, 0xc7, 0x18, 0x22};
  target.memory.install(algorithm, unknown_store);
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08RunClock clock(target.memory, bdc, timing_contract);
  const auto saved = clock.checkpoint();
  go(bdc);
  for (unsigned n = 0; n < 5; ++n) { clock.advance(); }
  require(rejected([&] { clock.advance(); }).contains("address=0x1822") && clock.faulted() &&
              target.cpu.faulted() && clock.state().executing_periods == 6 &&
              clock.state().retired_instructions == 1 &&
              rejected([&] { clock.restore_checkpoint(saved); }).contains("coordinated recovery"),
          "Actual native data faults retain elapsed periods and require joint recovery");
}

void prefixed_fetches() {
  using blofeld::test::hcs08::Memory;
  Memory memory;
  constexpr unsigned entry = 0x8000;
  constexpr unsigned prefix = 0x9e;
  constexpr unsigned load = 0xfe;
  constexpr unsigned save = 0xff;
  constexpr unsigned near_end = 0xfffe;
  constexpr unsigned last = 0xffff;
  for (const auto expected : std::array<std::array<unsigned, 2>, 8>{{{load, 5},
                                                                     {save, 5},
                                                                     {0xee, 4},
                                                                     {0xe1, 4},
                                                                     {0x68, 6},
                                                                     {0x69, 6},
                                                                     {0xeb, 4},
                                                                     {0xe9, 4}}}) {
    const std::array<std::uint8_t, 3> code{prefix, static_cast<std::uint8_t>(expected[0]), 1};
    memory.install(entry, code);
    require(hcs08_instruction_timing(memory, {.pc = entry}) ==
                Hcs08InstructionTiming{.bytes = 3, .periods = expected[1]},
            "All eight previously qualified secondary timing tuples remain exact");
  }
  constexpr std::array<std::uint8_t, 3> code{prefix, load, 1};
  for (unsigned missing = 1; missing <= 2; ++missing) {
    memory.install(entry, code);
    memory.known.reset(entry + missing);
    require(rejected([&] {
              static_cast<void>(hcs08_instruction_timing(memory, {.pc = entry}));
            }).contains("fetch address=" + hex(entry + missing)),
            "Unknown secondary opcode and displacement both fail before timing admission");
  }
  memory.install(near_end, std::span(code).first(2));
  for (const auto pc : {near_end, last}) {
    if (pc == last) { memory.install(last, std::span(code).first(1)); }
    require(rejected([&] {
              static_cast<void>(
                  hcs08_instruction_timing(memory, {.pc = static_cast<std::uint16_t>(pc)}));
            }).contains("instruction fetch wraps"),
            "Neither prefix decoding nor the full stack-relative fetch can wrap");
  }
}
} // namespace

int main() {
  try {
    costs_and_replay();
    multiply_and_call();
    decoder_and_unknowns();
    prefixed_fetches();
    quarantine();
    std::cout << "HCS08 documented instruction budgets, native boundaries, held time and replay "
                 "verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
