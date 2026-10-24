// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace access = blofeld::test::reference;
constexpr unsigned program = 0x100;
constexpr unsigned initial_r0 = 0x7e;
constexpr unsigned first_r0 = 0x12;
constexpr unsigned second_r0 = 0x34;
constexpr unsigned conditional_base = 0x0e0000; // FM13-80: 00001110 CCCC aaaaaaaaaaaa.
constexpr unsigned condition_shift = 12;
constexpr unsigned condition_count = 16;
constexpr unsigned ccr_count = 256;
constexpr unsigned ordinary_status = 0xc00300;

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

void fetch_words(DspMachine &machine, const std::size_t count) {
  for (std::size_t index = 0; index < count; ++index) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
  }
}

void execute_fresh(DspMachine &machine, const std::size_t words) {
  const auto progress = machine.begin_reference_instruction(access::boundaries);
  fetch_words(machine, words);
  static_cast<void>(
      access::finish(machine, machine.advance_reference_access(access::ticket(progress))));
}

unsigned
execute_condition(DspMachine &machine, const ConditionInput input, const unsigned previous_r0) {
  static const auto first_move = checkpoint::assemble("move #<$12,r0").front();
  static const auto second_move = checkpoint::assemble("move #<$34,r0").front();
  auto expected_r0 = previous_r0;
  const auto before = machine.cpu().debug_state();
  const auto opcode = conditional_base | (input.code << condition_shift) | (program + 3);
  machine.bus().write({.address = program + 1, .value = ordinary_status | input.ccr});
  machine.bus().write({.address = program + 2, .value = opcode});
  machine.bus().write({.address = program + 3});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  if (!machine.reference_fetch().active) {
    static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  }
  fetch_words(machine, 6);
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  progress = access::finish(machine, machine.advance_reference_access(access::ticket(progress)));
  require(progress.retired && machine.cpu().state().sr == (ordinary_status | input.ccr),
          "The native SR write supplies every literal CCR without arithmetic recomputation");
  const auto changed = expected_r0 == first_r0 ? second_r0 : first_r0;
  machine.bus().write(
      {.address = program + 3, .value = changed == first_r0 ? first_move : second_move});
  access::execute(machine);
  const bool taken = expected_condition(input);
  const auto fetch = machine.reference_fetch();
  require(machine.cpu().state().pc == program + 3 && fetch.used == (taken ? 0U : 3U) &&
              fetch.pending_address == (taken ? std::optional<unsigned>{} : program + 6) &&
              fetch.next_address == program + (taken ? 3U : 7U),
          "Actual same-address Jcc decision agrees with the independent full CCR truth matrix");
  if (taken) {
    execute_fresh(machine, 1);
    expected_r0 = changed;
    execute_fresh(machine, 2);
  } else {
    access::execute(machine);
    access::execute(machine);
  }
  const auto after = machine.cpu().debug_state();
  require(after.cpu.r[0] == expected_r0 && after.cpu.sr == (ordinary_status | input.ccr) &&
              after.cpu.pc == program && after.cpu.sp == 0 &&
              after.native_instructions == before.native_instructions + 4 &&
              machine.reference_fetch().used == 0 && !machine.reference_fetch().pending_address,
          "Literal flags/registers, four native retirements and loop redirect distinguish "
          "refetch from retained NOP");
  return expected_r0;
}

void condition_matrix() {
  DspMachine machine;
  DspProgram entry;
  entry.immediate("r0", initial_r0);
  entry.word(access::long_jump);
  entry.word(program);
  entry.install(machine.bus());
  const auto status = checkpoint::assemble("move #>$c00300,sr");
  require(status.size() == 2, "Independent full-width immediate SR instruction length");
  const std::array words{
      status.front(), status.back(), conditional_base, 0U, access::long_jump, program, 0U};
  checkpoint::ram(machine.bus(), program, words);
  checkpoint::run(machine, 2);
  unsigned expected_r0 = initial_r0;
  for (unsigned condition = 0; condition < condition_count; ++condition) {
    for (unsigned ccr = 0; ccr < ccr_count; ++ccr) {
      expected_r0 = execute_condition(machine, {.code = condition, .ccr = ccr}, expected_r0);
    }
  }
}
}

int main() {
  try {
    condition_matrix();
    std::cout << "4096 native same-address Jcc/CCR cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
