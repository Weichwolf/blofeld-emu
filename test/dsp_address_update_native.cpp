// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_address_update_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <iostream>

namespace blofeld::test::address_update {
namespace {
void one(const Case input, const unsigned reg) {
  const auto word = opcode(input, reg);
  DspMachine machine;
  const auto before = prepare(machine, input, reg);
  auto expected = before;
  expected.cpu.r.at(reg) = input.expected;
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  const auto saved = machine.checkpoint();
  const auto execute = [&](DspMachine &branch) {
    unsigned before_callbacks = 0;
    unsigned after_callbacks = 0;
    branch.cpu().step({.observe = [&](const DspNativeAccess &access) {
      require(access.kind == DspNativeAccessKind::opcode && access.target.space == DspSpace::p &&
                  access.target.address == entry && access.pc == entry && access.ordinal == 0,
              "A U instruction has only its actual opcode access and no data bus callback");
      if (access.stage == DspNativeAccessStage::before) {
        ++before_callbacks;
        require(!access.value, "The opcode before callback carries no invented result");
      } else {
        ++after_callbacks;
        require(access.value == word, "The opcode after callback contains the actual word");
      }
    }});
    require(
        before_callbacks == 1 && after_callbacks == 1 && branch.cpu().debug_state() == expected,
        "U updates only the selected R register/PC/counters and leaves all CCR/CPU fields intact");
  };
  execute(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    DspMachine replay;
    replay.restore(saved);
    require(replay.cpu().debug_state() == before, "The snapshot preserves every native input");
    execute(replay);
  }
}
}

void native_checks() {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (const auto input : cases) { one(input, reg); }
  }
  std::cout << "192 native U cases,384 independent restored suffixes; no data bus callbacks\n";
}
}
