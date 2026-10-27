// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <variant>

namespace blofeld {
namespace {
constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

DspReferenceProgress
extension(DspMachine &machine, const DspReferenceProgress &progress, const DspProgramWord word) {
  if (!progress.pending) { throw Fault("Automatic E1 lost its native extension request"); }
  const auto *access = std::get_if<DspNativeAccess>(&progress.pending->effect);
  if (access == nullptr || access->kind != DspNativeAccessKind::extension ||
      access->target.address != word.address ||
      progress.pending->latched_program_word != word.value) {
    throw Fault("Automatic E1 extension differs from actual AG1/F2 at P:" + hex(word.address, 6));
  }
  return machine.advance_reference_access(progress.pending->ticket);
}
}

DspReferenceProgress DspInstructionPipeline::admit_opcode(DspMachine &machine,
                                                          const DspReferencePending &pending,
                                                          const DspProgramWord word) {
  const auto *access = std::get_if<DspNativeAccess>(&pending.effect);
  DspReferenceProgress progress{.pending = pending};
  if (access != nullptr && access->kind == DspNativeAccessKind::opcode &&
      access->target.address == word.address && pending.latched_program_word == word.value) {
    progress = machine.advance_reference_access(pending.ticket);
  } else if (!machine.pipeline_repeat_result() || machine.pipeline_repeat_body() != samples[2] ||
             !std::holds_alternative<DspInstructionRetire>(pending.effect)) {
    throw Fault("Automatic DSP E1 opcode does not match its actual native/F2 latch");
  }
  return progress;
}

void DspInstructionPipeline::execute(DspMachine &machine) {
  if (!view.cycle.execute1) { throw Fault("Automatic DSP E1 lost its phase word"); }
  const auto word = *view.cycle.execute1;
  view.cycle.candidate_inputs = machine.cpu().state();
  auto pending = machine.reference_pending();
  if (!pending) {
    const auto progress = machine.begin_reference_instruction(execution);
    pending = progress.pending;
  }
  if (!pending) { throw Fault("Automatic DSP E1 lost its actual opcode boundary"); }
  auto progress = admit_opcode(machine, *pending, word);
  if (view.cycle.e1_extension) {
    progress = extension(machine, progress, *view.cycle.e1_extension);
  }
  const auto port_read = decode_peripheral_read(word.value);
  if (port_read) {
    const auto ticket = reads[1];
    if (!ticket || !progress.pending) {
      throw Fault("Peripheral E1 lost its owned actual data boundary");
    }
    progress = machine.advance_pipeline_peripheral_read(*ticket, progress.pending->ticket);
  }
  const auto fields =
      port_read ? decode_move_writeback(word.value) : pipeline_transfer_fields(word.value);
  if (fields != DspMoveFields{}) {
    if (!progress.pending || !std::holds_alternative<DspMoveWriteback>(progress.pending->effect)) {
      throw Fault("Automatic transfer E1 lost its actual native load fields");
    }
    progress = machine.advance_reference_access(progress.pending->ticket);
    if (!progress.completed) { throw Fault("Automatic transfer E1 lost its real publication"); }
    const auto *move = std::get_if<DspMoveWriteback>(&*progress.completed);
    if (move == nullptr || move->fields != fields || move->pc != word.address) {
      throw Fault("Automatic transfer E1 differs from its admitted native destination");
    }
    view.cycle.transfer = *move;
  }
  if (!progress.pending || progress.retired ||
      !std::holds_alternative<DspInstructionRetire>(progress.pending->effect)) {
    throw Fault("Unqualified actual native effect in automatic DSP E1");
  }
  if (decode_branch_form(word.value) || (port_read && port_read->bit)) {
    validate_branch(std::get<DspInstructionRetire>(progress.pending->effect));
  }
  view.cycle.execution = progress;
  view.cycle.execution_captured = true;
  view.awaiting_retirement = view.cycle.execute1;
  view.awaiting_inputs = view.cycle.candidate_inputs;
  view.awaiting_deferred_accumulators = view.cycle.e1_deferred_accumulators;
}
}
