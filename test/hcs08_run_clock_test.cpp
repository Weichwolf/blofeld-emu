// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_run_clock.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
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

void decoder_and_unknowns() {
  using blofeld::test::hcs08::Memory;
  constexpr unsigned entry = 0x8000;
  constexpr unsigned flags = 0x68;
  constexpr unsigned values = 256;

  struct Expected {
    unsigned opcode, bytes, periods;
  };

  constexpr std::array<Expected, 6> documented{{{.opcode = 0xae, .bytes = 2, .periods = 2},
                                                {.opcode = 0xa6, .bytes = 2, .periods = 2},
                                                {.opcode = 0xc7, .bytes = 3, .periods = 4},
                                                {.opcode = 0x94, .bytes = 1, .periods = 2},
                                                {.opcode = 0x9d, .bytes = 1, .periods = 1},
                                                {.opcode = 0x82, .bytes = 1, .periods = 5}}};
  Memory memory;
  unsigned qualified = 0;
  unsigned rejected_forms = 0;
  for (unsigned opcode = 0; opcode < values; ++opcode) {
    const std::array<std::uint8_t, 3> code{static_cast<std::uint8_t>(opcode), 0, store};
    memory.install(entry, code);
    bool known = false;
    for (const auto expected : documented) {
      if (expected.opcode != opcode) { continue; }
      require(hcs08_instruction_timing(memory, {.pc = entry, .ccr = flags}) ==
                  Hcs08InstructionTiming{.bytes = expected.bytes, .periods = expected.periods},
              "Six opcode-form lengths/costs independently transcribed from the primary table");
      known = true;
      ++qualified;
    }
    if (!known) {
      const auto reason = rejected([&] {
        static_cast<void>(hcs08_instruction_timing(memory, {.pc = entry, .ccr = flags}));
      });
      require(reason.contains("unsupported opcode") && reason.contains("address=0x8000") &&
                  reason.contains("ccr=0x0068"),
              "Unimplemented timing forms cannot acquire guessed costs");
      ++rejected_forms;
    }
  }
  constexpr unsigned unsupported = 250;
  require(qualified == documented.size() && rejected_forms == unsupported,
          "Every opcode form has an explicit qualified/unsupported timing result");
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
} // namespace

int main() {
  try {
    costs_and_replay();
    decoder_and_unknowns();
    quarantine();
    std::cout << "HCS08 documented instruction budgets, native boundaries, held time and replay "
                 "verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
