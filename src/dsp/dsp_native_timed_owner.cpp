// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

namespace blofeld {
bool DspNativeTimedExecution::run_owner(dsp56300::Core &cpu,
                                        const dsp56300::Io & /*code*/,
                                        const DspTimedInstruction accounted) {
  const auto prepared = prepare_owner(cpu, accounted.words, accounted.base_units);
  if (prepared.dependency_units != accounted.dependency_units) {
    throw Fault("DSP captured owner changed its reserved dependency clocks pc=" +
                hex(accounted.words.address, 6));
  }
  const auto shape = dsp56300::inspect_instruction(accounted.words.opcode);
  using Kind = dsp56300::InstructionKind;
  if (!shape ||
      (shape->kind != Kind::parallel && shape->kind != Kind::nop &&
       shape->kind != Kind::move_peripheral_register) ||
      cpu.instruction_fetch().exception_vector) {
    return false;
  }
  const auto actual = cpu.run_instruction(accounted.words);
  if (actual.fault.kind != dsp56300::FaultKind::none) { return false; }
  const auto after = cpu.progress();
  if (actual.dispatches != 1 || actual.progress != after ||
      after.instructions < expected_.instructions ||
      after.estimated_cycles < expected_.estimated_cycles ||
      after.instructions - expected_.instructions != 1 ||
      after.estimated_cycles - expected_.estimated_cycles != accounted.base_units) {
    throw Fault("DSP captured owner lost its actual single-instruction retirement pc=" +
                hex(accounted.words.address, 6));
  }
  commit_owner(accounted, after, false);
  return true;
}
}
