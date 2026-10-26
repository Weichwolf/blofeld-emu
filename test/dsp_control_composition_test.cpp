// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_control_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned body = boot + 4;
constexpr unsigned target = body + 20;
constexpr unsigned jump_r1 = 0x0ae180;
constexpr unsigned clear_a = 0x200013;
constexpr unsigned short_bra_13 = 0x050c0d;

struct Result {
  unsigned pc;
  unsigned next;
  unsigned sr;
  unsigned cost;
};

void one_case(const std::vector<std::uint32_t> &words,
              const std::span<const Result> results,
              const unsigned last) {
  DspMachine machine;
  const std::array seeds{control::sr_move, masked_status, 0x61f400U, target};
  auto expected = control::prepare(machine, seeds, 2, words);
  DspFetchRun run(control::contract(), machine);
  unsigned retired = 0;
  while (run.completed_cycles() < last) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if (!phase.retirement_published) { continue; }
    require(retired < results.size(), "Only the declared literal path retires");
    const auto result = results[retired];
    require(phase.execute2 && phase.execute2->address == result.pc && expected.cpu.pc == result.pc,
            "Control/CCR composition retains the real native path and once-only ordering");
    expected.cpu.pc = result.next;
    expected.cpu.sr = result.sr;
    ++expected.native_instructions;
    expected.native_cycles += result.cost;
    ++retired;
    control::check(machine, expected);
  }
  require(retired == results.size(),
          "Independent unchanged-model composition window includes the true final NOP");
}

Result nop(const unsigned pc) {
  return {.pc = pc, .next = pc + 1, .sr = masked_status, .cost = 1};
}

void combinations() {
  std::vector<std::uint32_t> words(control::padding, 0);
  words.at(1) = control::and_ccr;
  words.at(2) = jump_r1;
  const std::array and_jump{
      nop(body),
      Result{.pc = body + 1, .next = body + 2, .sr = masked_status, .cost = 3},
      Result{.pc = body + 2, .next = target, .sr = masked_status, .cost = 3},
      nop(target)};
  one_case(words, and_jump, 14);
  words.at(1) = jump_r1;
  words.at(2) = 0;
  words.at(target - body) = control::and_ccr;
  const std::array jump_and{
      nop(body),
      Result{.pc = body + 1, .next = target, .sr = masked_status, .cost = 3},
      Result{.pc = target, .next = target + 1, .sr = masked_status, .cost = 3},
      nop(target + 1)};
  one_case(words, jump_and, 14);
  words.at(target - body) = 0;
  words.at(1) = control::and_ccr;
  words.at(2) = short_bra_13;
  const std::array and_bra{
      nop(body),
      Result{.pc = body + 1, .next = body + 2, .sr = masked_status, .cost = 3},
      Result{.pc = body + 2, .next = body + 15, .sr = masked_status, .cost = 4},
      nop(body + 15)};
  one_case(words, and_bra, 15);
  words.at(1) = clear_a;
  words.at(2) = control::and_ccr;
  const std::array alu_and{
      nop(body),
      Result{.pc = body + 1, .next = body + 2, .sr = masked_status | 0x14U, .cost = 1},
      Result{.pc = body + 2, .next = body + 3, .sr = masked_status, .cost = 3},
      nop(body + 3)};
  one_case(words, alu_and, 12);
  words.at(1) = jump_r1;
  words.at(2) = 0;
  words.at(target - body) = jump_r1;
  const std::array adjacent_jumps{
      nop(body),
      Result{.pc = body + 1, .next = target, .sr = masked_status, .cost = 3},
      Result{.pc = target, .next = target, .sr = masked_status, .cost = 3},
      Result{.pc = target, .next = target, .sr = masked_status, .cost = 3}};
  one_case(words, adjacent_jumps, 14);
}
}

int main() {
  try {
    combinations();
    std::cout << "Five native CCR/ALU/short-branch/indirect-control compositions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
