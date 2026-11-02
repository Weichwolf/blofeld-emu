// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_cycle_cache_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <iostream>
#include <tuple>

namespace blofeld::test {
namespace {
using esai::require;
namespace native = dma_test;
constexpr unsigned site = 0x100, payload = 0x123456, loop_end = 0x123;
constexpr unsigned core_priority = 0xffffff, peripheral_priority = 0xfffffe;
constexpr unsigned modifier = 7, masked_status = 0xc00300;
// FM13 MOVEC #imm,M0/SR/LA, JMP abs, MOVE #imm,X0 and NOP.
constexpr unsigned move_m0 = 0x05f420, move_sr = 0x05f439, move_la = 0x05f43e;
constexpr unsigned jump = 0x0af080, immediate_x0 = 0x44f400;
constexpr unsigned absolute_jump_units = 4; // FM A-1/A-4: JMP EA = 3 plus 1 for long absolute.

auto observe(const OwnDspMachine &machine) {
  return std::tuple{machine.cpu().registers(),
                    machine.cpu().progress(),
                    machine.cpu().fault(),
                    machine.cpu().processing_state(),
                    machine.clock().elapsed_ticks()};
}

void no_jit(const OwnDspMachine &machine) {
  const auto stats = machine.cpu().jit_statistics();
  require(stats.compiled_blocks == 0 && stats.executable_bytes == 0 &&
              stats.native_alu_instructions == 0 && stats.native_move_instructions == 0 &&
              stats.batch_instructions == 0,
          "External native execution, mutation and restore do not create executable code");
}

void step(OwnDspMachine &machine, const unsigned pc, const unsigned cycles) {
  const auto before = observe(machine);
  const auto result = machine.run_unclocked();
  const auto after = observe(machine);
  std::cout << "STEP pc=" << hex(std::get<0>(before).pc, 6)
            << " actual-next=" << hex(std::get<0>(after).pc, 6) << " expected-next=" << hex(pc, 6)
            << " actual-units="
            << std::get<1>(after).estimated_cycles - std::get<1>(before).estimated_cycles
            << " expected-units=" << cycles << " retired=" << result.dispatches
            << " fault=" << static_cast<unsigned>(result.fault.kind) << '\n';
  require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::none &&
              std::get<0>(after).pc == pc &&
              std::get<1>(after).instructions == std::get<1>(before).instructions + 1 &&
              std::get<1>(after).estimated_cycles ==
                  std::get<1>(before).estimated_cycles + cycles &&
              std::get<4>(after) == std::get<4>(before),
          "Actual native retirement has the independent PC/base cost without invented clocks");
  no_jit(machine);
}

void prepare(OwnDspMachine &machine) {
  native::NativeProgram program;
  program.word(0); // NOP at the original high ROM entry, before the literal register seeds.
  for (const auto word :
       {move_m0, modifier, move_sr, masked_status, move_la, loop_end, jump, site}) {
    program.word(word);
  }
  native::reset_native(machine, program);
  // Authored mutable RAM: NOP, NOP, JMP site. Immediate MOVE replaces the first pair.
  const std::array body{0U, 0U, jump, site};
  for (unsigned index = 0; index < body.size(); ++index) {
    machine.bus().write({.address = site + index, .value = body[index]});
  }
  auto expected = observe(machine);
  std::get<0>(expected).pc = native::boot + 1;
  ++std::get<1>(expected).instructions;
  ++std::get<1>(expected).estimated_cycles;
  step(machine, native::boot + 1, 1);
  require(observe(machine) == expected, "High-ROM NOP retains every register and flag");
  step(machine, native::boot + 3, 2);
  step(machine, native::boot + 5, 2);
  step(machine, native::boot + 7, 2);
  step(machine, site, absolute_jump_units);
  const auto regs = machine.cpu().registers();
  require(regs.m[0] == modifier && regs.sr == masked_status && regs.la == loop_end,
          "Actual immediate register instructions retain raw M0, SR and LA values");
}

void check_priorities(OwnDspMachine &machine) {
  require(machine.bus().read({.space = DspSpace::x, .address = core_priority}) == 0 &&
              machine.bus().read({.space = DspSpace::x, .address = peripheral_priority}) == 0,
          "Execution and joint restore preserve actual reset interrupt priorities");
}
}

void native_cycle_cache() {
  OwnDspMachine machine(native::native_clocks);
  prepare(machine);
  check_priorities(machine);
  const auto before = observe(machine);
  const auto saved = machine.checkpoint();
  step(machine, site + 1, 1);
  auto expected = before;
  std::get<0>(expected).pc = site + 1;
  ++std::get<1>(expected).instructions;
  ++std::get<1>(expected).estimated_cycles;
  require(observe(machine) == expected,
          "NOP at the declared RAM location changes only PC and independent progress counts");
  const auto nop = observe(machine);
  step(machine, site + 2, 1);
  step(machine, site, absolute_jump_units);
  const auto changed_from = observe(machine);
  machine.bus().write({.address = site, .value = immediate_x0});
  machine.bus().write({.address = site + 1, .value = payload});
  step(machine, site + 2, 2);
  expected = changed_from;
  std::get<0>(expected).pc = site + 2;
  std::get<0>(expected).x = payload;
  ++std::get<1>(expected).instructions;
  std::get<1>(expected).estimated_cycles += 2;
  require(observe(machine) == expected,
          "A changed opcode/operand retires MOVE with two base cycles and exact native state");
  machine.restore(saved);
  require(observe(machine) == before, "Joint restore recovers original RAM and full CPU state");
  step(machine, site + 1, 1);
  require(observe(machine) == nop, "Restored NOP cannot reuse the mutated MOVE cost or operand");
  check_priorities(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    OwnDspMachine branch(native::native_clocks);
    branch.restore(saved);
    require(observe(branch) == before, "Fresh owners restore native program/progress together");
    step(branch, site + 1, 1);
    require(observe(branch) == nop, "Both fresh suffixes retain the original one-cycle NOP");
    check_priorities(branch);
  }
}
}
