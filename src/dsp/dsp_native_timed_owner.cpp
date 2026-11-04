// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <optional>

namespace blofeld {
namespace {
bool same_words(const dsp56300::InstructionWords &first, const dsp56300::InstructionWords &second) {
  return first.address == second.address && first.opcode == second.opcode &&
         first.extension == second.extension;
}
}

bool DspNativeTimedExecution::run_owner(dsp56300::Core &cpu,
                                        const dsp56300::Io &code,
                                        const DspTimedInstruction accounted) {
  const auto prepared = prepare_owner(cpu, accounted.words, accounted.base_units);
  if (prepared.dependency_units != accounted.dependency_units) {
    throw Fault("DSP bounded owner changed its reserved dependency clocks pc=" +
                hex(accounted.words.address, 6));
  }
  const auto shape = dsp56300::inspect_instruction(accounted.words.opcode);
  using Kind = dsp56300::InstructionKind;
  if (!shape || (shape->kind != Kind::parallel && shape->kind != Kind::nop) ||
      cpu.instruction_fetch().exception_vector) {
    return false;
  }
  const auto revision = code.program_revision();
  const auto preview = cpu.run(dsp56300::RunBudget{});
  if (code.program_revision() != revision) {
    throw Fault("DSP bounded owner code changed during its actual preview");
  }
  if (!preview.next || !same_words(*preview.next, accounted.words) ||
      preview.next_cycles != accounted.base_units) {
    return false;
  }
  const auto actual =
      cpu.run(dsp56300::RunBudget{.instructions = 1, .cycles = accounted.base_units});
  if (code.program_revision() != revision) {
    throw Fault("DSP bounded owner code/mapping changed while executing");
  }
  if (actual.execution.fault.kind != dsp56300::FaultKind::none) { return false; }
  if (actual.execution.dispatches == 0) {
    check_progress(cpu);
    if (actual.cycles != 0) { throw Fault("Unretired DSP bounded owner consumed CPU periods"); }
    return false;
  }
  const auto after = cpu.progress();
  if (actual.execution.dispatches != 1 || actual.execution.progress != after ||
      actual.cycles != accounted.base_units || after.instructions < expected_.instructions ||
      after.estimated_cycles < expected_.estimated_cycles ||
      after.instructions - expected_.instructions != 1 ||
      after.estimated_cycles - expected_.estimated_cycles != accounted.base_units) {
    throw Fault("DSP bounded owner lost its actual single-instruction retirement pc=" +
                hex(accounted.words.address, 6));
  }
  commit_owner(accounted, after, false);
  return true;
}
}
