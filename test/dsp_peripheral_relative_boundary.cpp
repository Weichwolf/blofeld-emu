// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_relative_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace blofeld::test::peripheral_relative {
namespace {
// FM5.4.4.1: opcode, extension and sequential/redirected PC addresses remain24-bit.
void program_boundary(const unsigned operation, const bool flags) {
  constexpr unsigned move_immediate_x0 = 0x44f400;
  constexpr unsigned literal = 0x456789;
  constexpr unsigned jump_target = 0x20;
  DspMachine machine;
  auto expected = prepare(machine, {.flags = flags, .start = word_mask});
  constexpr std::array opcodes{0U, move_immediate_x0, long_jump};
  constexpr std::array next{0U, 1U, jump_target};
  const auto opcode_word = opcodes.at(operation);
  store_word(machine.bus(), {.address = word_mask, .value = opcode_word});
  store_word(machine.bus(), {.address = 0, .value = operation == 1 ? literal : jump_target});
  if (operation == 0) { store_word(machine.bus(), {.address = 0, .value = 0}); }
  store_word(machine.bus(), {.address = operation == 2 ? jump_target : 1, .value = 0});
  expected.cpu.pc = next.at(operation);
  if (operation == 1) { expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | literal; }
  ++expected.native_instructions;
  // FM A-1: baseline NOP1, immediate MOVE1+lim1, long-absolute JMP3+lab1.
  constexpr std::array nominal_cycles{1U, 2U, 4U};
  expected.native_cycles += nominal_cycles.at(operation);
  const auto saved = machine.checkpoint();
  for (unsigned repeat = 0; repeat < 3; ++repeat) {
    if (repeat != 0) { machine.restore(saved); }
    unsigned extensions = 0;
    machine.cpu().step({.observe = [&](const DspNativeAccess &access) {
      require(access.target.space == DspSpace::p && access.target.address <= word_mask,
              "Architectural PC wrapping must precede native bus access, not hide an invalid bus "
              "address");
      if (access.kind == DspNativeAccessKind::extension) {
        ++extensions;
        require(access.target.address == 0 && access.pc == word_mask,
                "The actual extension is fetched at P000000 with opcode context Pffffff");
      }
    }});
    require(extensions == (operation == 0 ? 0U : 2U) && machine.cpu().debug_state() == expected,
            "Boundary NOP/immediate MOVE/absolute JMP agrees with independent complete state");
    auto after = expected;
    after.cpu.pc = (after.cpu.pc + 1) & word_mask;
    ++after.native_instructions;
    ++after.native_cycles;
    machine.cpu().step();
    require(machine.cpu().debug_state() == after,
            "The next actual opcode fetch must consume the correctly wrapped architectural PC");
  }
}

void repeat_boundary(const unsigned count, const bool flags) {
  constexpr unsigned repeat = 0x0600a0;
  constexpr unsigned count_shift = 8;
  DspMachine machine;
  auto expected = prepare(machine, {.flags = flags, .start = word_mask - 1});
  store_word(machine.bus(), {.address = word_mask - 1, .value = repeat | (count << count_shift)});
  store_word(machine.bus(), {.address = word_mask, .value = 0});
  store_word(machine.bus(), {.address = 0, .value = 0});
  const auto original_lc = expected.cpu.lc;
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 3; ++replay) {
    machine.restore(saved);
    auto state = expected;
    state.cpu.pc = count == 0 ? 0U : word_mask;
    state.cpu.lc = count == 0 ? original_lc : count;
    ++state.native_instructions;
    state.native_cycles += 5;
    machine.cpu().step();
    require(machine.cpu().debug_state() == state,
            "Boundary REP prefix uses the actual24-bit body address");
    for (unsigned body = 0; body < count; ++body) {
      const auto remaining = count - body - 1;
      state.cpu.pc = remaining == 0 ? 0U : word_mask;
      state.cpu.lc = remaining == 0 ? original_lc : remaining;
      ++state.native_instructions;
      ++state.native_cycles;
      machine.cpu().step();
      require(machine.cpu().debug_state() == state,
              "Each real boundary REP body retains full native state");
    }
    state.cpu.pc = 1;
    ++state.native_instructions;
    ++state.native_cycles;
    machine.cpu().step();
    require(machine.cpu().debug_state() == state,
            "Boundary REP resumes at the actual wrapped successor");
  }
}

void loop_boundary(const unsigned count, const bool flags) {
  constexpr unsigned begin = word_mask - 3;
  constexpr unsigned body = begin + 2;
  constexpr unsigned immediate_do = 0x060080;
  constexpr unsigned count_shift = 8;
  DspMachine machine;
  const auto before = prepare(machine, {.flags = flags, .start = begin});
  store_word(machine.bus(), {.address = begin, .value = immediate_do | (count << count_shift)});
  store_word(machine.bus(), {.address = begin + 1, .value = word_mask});
  store_word(machine.bus(), {.address = body, .value = 0});
  store_word(machine.bus(), {.address = word_mask, .value = 0});
  store_word(machine.bus(), {.address = 0, .value = 0});
  auto expected = before;
  expected.cpu.pc = 1;
  expected.stack_high.at(1) = before.cpu.la;
  expected.stack_low.at(1) = before.cpu.lc;
  expected.stack_high.at(2) = body;
  expected.stack_low.at(2) = before.cpu.sr;
  expected.native_instructions += 2 + 2 * count;
  expected.native_cycles += 6 + 2 * count;
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 3; ++replay) {
    machine.restore(saved);
    machine.cpu().step();
    for (unsigned iteration = 0; iteration < count; ++iteration) {
      require(machine.cpu().state().pc == body,
              "A real DO backedge selects the first wrapped loop word");
      machine.cpu().step();
      require(machine.cpu().state().pc == word_mask, "The real final DO body word is Pffffff");
      machine.cpu().step();
    }
    require(machine.cpu().state().pc == 0,
            "DO completion/annulment selects P000000 before the next fetch");
    machine.cpu().step();
    require(machine.cpu().debug_state() == expected,
            "Boundary DO restores complete CPU/counters/physical stack");
  }
}
}

void boundary_checks() {
  for (const bool flags : {false, true}) {
    for (unsigned operation = 0; operation < 3; ++operation) { program_boundary(operation, flags); }
    for (const unsigned count : {0U, 1U, 3U}) {
      repeat_boundary(count, flags);
      loop_boundary(count, flags);
    }
  }
  std::cout << "18 NOP/MOVE/JMP/REP/DO boundary windows; two restored suffixes each\n";
}
}
