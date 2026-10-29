// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_accumulator_fixture.hpp"
#include "dsp_writeback_fixture.hpp"

#include <iostream>

namespace blofeld::test::peripheral_accumulator {
namespace {
void one(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  auto expected = before;
  load_result(expected, input);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 3; ++replay) {
    if (replay != 0) { machine.restore(saved); }
    unsigned reads = 0;
    machine.cpu().step({.observe = [&](const DspNativeAccess &access) {
      require(access.pc == entry && access.omr == ordinary_omr && !access.bit,
              "MOVEP retains its real opcode/OMR execution context");
      if (access.kind == DspNativeAccessKind::opcode) { return; }
      const auto port = ports.at(input.port);
      require(access.kind == DspNativeAccessKind::read && access.target.space == port.space &&
                  access.target.address == port.address && access.ordinal == 1,
              "Full-accumulator MOVEP has one actual word read and no fabricated write/extension");
      ++reads;
      if (access.stage == DspNativeAccessStage::after) {
        require(access.value == source_value(input),
                "Native load consumes the actual peripheral word");
      }
    }});
    require(reads == 2 && machine.cpu().debug_state() == expected,
            "Independent full56 sign extension/zero-fill/unchanged CCR/CPU/counter expectations "
            "differ");
  }
}

void bare_stage(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  const auto fields = input.b ? DspMoveFields{.b = 7} : DspMoveFields{.a = 7};
  require(decode_move_writeback(opcode(input)) == fields,
          "The independent full destination mask contains exactly all three A or B fields");
  static_cast<void>(machine.begin_reference_instruction(writeback::execution));
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    const auto error = checkpoint::rejects(
        [&] {
          static_cast<void>(
              writeback::await_move(machine, {.pending = machine.reference_pending()}));
        },
        "Bare staged MOVEP must not bypass its actual automatic AG1 response owner");
    require(error.contains("Unqualified native memory-address phase read") &&
                error.contains("pc=") && error.contains("omr=") &&
                machine.cpu().debug_state() == before,
            "Full accumulator admission preserves strict source-owner protection before any native "
            "result");
  }
}
}

void native_checks() {
  matrix([](const Input input) { one(input); });
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (const bool b : {false, true}) {
      for (const auto value : {positive_edge, negative_edge}) {
        bare_stage({.port = port, .value = value, .status = masked_status | all_ccr, .b = b});
      }
    }
  }
  std::cout << "768 native full loads/1536 suffixes;16 bare-stage guards/32 restored refusals "
               "passed\n";
}
}
