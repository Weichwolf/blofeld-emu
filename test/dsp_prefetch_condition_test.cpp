// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
namespace native = blofeld::test::dma_test;
using blofeld::test::esai::require;
constexpr unsigned program = 0x100;
constexpr unsigned initial_r0 = 0x7e;
constexpr unsigned first_r0 = 0x12;
constexpr unsigned second_r0 = 0x34;
constexpr unsigned conditional_base = 0x0e0000; // FM13-80: 00001110 CCCC aaaaaaaaaaaa.
constexpr unsigned condition_shift = 12;
constexpr unsigned condition_count = 16;
constexpr unsigned ccr_count = 256;
constexpr unsigned ordinary_status = 0xc00300;
constexpr unsigned carry = 1;
constexpr unsigned overflow = 2;
constexpr unsigned zero = 4;
constexpr unsigned negative = 8;
constexpr unsigned unnormalized = 0x10;
constexpr unsigned extension = 0x20;
constexpr unsigned limit = 0x40;
// FM MOVE immediate R0/full SR and absolute-extension JMP.
constexpr unsigned long_r0 = 0x60f400;
constexpr unsigned short_first = 0x301200;
constexpr unsigned short_second = 0x303400;
constexpr unsigned long_sr = 0x05f439;
constexpr unsigned long_jump = 0x0af080;
constexpr unsigned effect_bound = 32;

struct ConditionInput {
  unsigned code;
  unsigned ccr;
};

bool expected_condition(const ConditionInput input) {
  const auto ccr = input.ccr;
  // Independent FM12.5.1/Table12-17 equations and Table12-18 bit encoding.
  const bool c = (ccr & carry) != 0;
  const bool v = (ccr & overflow) != 0;
  const bool z = (ccr & zero) != 0;
  const bool n = (ccr & negative) != 0;
  const bool u = (ccr & unnormalized) != 0;
  const bool e = (ccr & extension) != 0;
  const bool l = (ccr & limit) != 0;
  const std::array predicates{!c,
                              n == v,
                              !z,
                              !n,
                              !z && (u || e),
                              !e,
                              !l,
                              !z && n == v,
                              c,
                              n != v,
                              z,
                              n,
                              z || (!u && !e),
                              e,
                              l,
                              z || n != v};
  return predicates.at(input.code);
}

void fetch_words(OwnDspFetch &fetch, const std::size_t count) {
  for (std::size_t index = 0; index < count; ++index) {
    fetch.issue();
    fetch.capture();
  }
}

void execute(OwnDspMachine &machine, OwnDspFetch &fetch, const bool restore = false) {
  const auto before = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  auto progress = fetch.begin();
  for (unsigned effect = 0; effect < effect_bound && progress.pending && !progress.retired &&
                            progress.fault.kind == dsp56300::FaultKind::none;
       ++effect) {
    progress = machine.advance(progress.pending->ticket);
  }
  require(progress.retired && progress.fault.kind == dsp56300::FaultKind::none &&
              machine.cpu().progress().instructions == before + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "Exactly one actual native instruction retires without inventing clock periods");
  if (restore) {
    OwnDspMachine restored(native::native_clocks);
    OwnDspFetch replay(restored, fetch.checkpoint());
    replay.synchronize();
    fetch.synchronize();
    require(
        replay.view() == fetch.view() && restored.cpu().registers() == machine.cpu().registers() &&
            restored.cpu().progress() == machine.cpu().progress() &&
            restored.clock().elapsed_ticks() == ticks,
        "Post-retirement restoration retains the real branch decision before F2 synchronization");
  } else {
    fetch.synchronize();
  }
}

void execute_fresh(OwnDspMachine &machine, OwnDspFetch &fetch, const std::size_t words) {
  fetch_words(fetch, words);
  execute(machine, fetch);
}

unsigned execute_condition(OwnDspMachine &machine,
                           OwnDspFetch &frontend,
                           const ConditionInput input,
                           const unsigned previous_r0) {
  auto expected_r0 = previous_r0;
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto opcode = conditional_base | (input.code << condition_shift) | (program + 3);
  machine.bus().write({.address = program + 1, .value = ordinary_status | input.ccr});
  machine.bus().write({.address = program + 2, .value = opcode});
  machine.bus().write({.address = program + 3});
  fetch_words(frontend, 6);
  frontend.issue();
  execute(machine, frontend);
  require(machine.cpu().registers().sr == (ordinary_status | input.ccr),
          "Native MOVEC SR supplies every literal CCR without arithmetic reenactment");
  const auto changed = expected_r0 == first_r0 ? second_r0 : first_r0;
  machine.bus().write(
      {.address = program + 3, .value = changed == first_r0 ? short_first : short_second});
  constexpr unsigned snapshot_stride = 64;
  execute(machine, frontend, input.ccr % snapshot_stride == 0);
  const bool taken = expected_condition(input);
  const auto fetch = frontend.view();
  require(machine.cpu().registers().pc == program + 3 && fetch.used == (taken ? 0U : 3U) &&
              fetch.pending_address == (taken ? std::optional<unsigned>{} : program + 6) &&
              fetch.next_address == program + (taken ? 3U : 7U),
          "Same-address Jcc must retain the independent branch decision: code=" +
              std::to_string(input.code) + " ccr=" + std::to_string(input.ccr) +
              " taken=" + (taken ? "true" : "false") + " queued=" + std::to_string(fetch.used));
  if (taken) {
    execute_fresh(machine, frontend, 1);
    expected_r0 = changed;
    execute_fresh(machine, frontend, 2);
  } else {
    execute(machine, frontend);
    execute(machine, frontend);
  }
  auto expected = before;
  expected.r[0] = expected_r0;
  expected.sr = ordinary_status | input.ccr;
  expected.pc = program;
  require(
      machine.cpu().registers() == expected && expected.sp == 0 &&
          machine.cpu().progress().instructions == progress.instructions + 4 &&
          frontend.view().used == 0 && !frontend.view().pending_address,
      "Independent registers/CCR, four retirements and real JMP distinguish refetch from held NOP");
  return expected_r0;
}

void condition_matrix() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram entry;
  entry.word(long_r0);
  entry.word(initial_r0);
  entry.word(long_jump);
  entry.word(program);
  native::reset_native(machine, entry);
  const std::array words{long_sr, ordinary_status, conditional_base, 0U, long_jump, program, 0U};
  for (unsigned i = 0; i < words.size(); ++i) {
    machine.bus().write({.address = program + i, .value = words[i]});
  }
  for (unsigned i = 0; i < 2; ++i) { native::step_native(machine); }
  OwnDspFetch frontend(machine);
  unsigned expected_r0 = initial_r0;
  for (unsigned condition = 0; condition < condition_count; ++condition) {
    for (unsigned ccr = 0; ccr < ccr_count; ++ccr) {
      expected_r0 =
          execute_condition(machine, frontend, {.code = condition, .ccr = ccr}, expected_r0);
    }
  }
}
}

int main() {
  try {
    condition_matrix();
    std::cout << "4096 native same-address Jcc/CCR cases and 64 retirement restores passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
