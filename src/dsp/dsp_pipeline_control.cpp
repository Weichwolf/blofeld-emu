// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>
#include <optional>
#include <variant>

namespace blofeld {
bool pipeline_ccr_immediate(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  constexpr unsigned ccr = 1;
  return opcodes.getInstructionTypes(opcode, first, second) != 0 && first == dsp56k::Andi &&
         second == dsp56k::Invalid && dsp56k::getFieldValue(first, dsp56k::Field_EE, opcode) == ccr;
}

bool DspInstructionPipeline::hold_ccr_decode(const DspMachine &machine) {
  if (!ccr_sync) { return false; }
  if (ccr_sync->decode_hold == 0) {
    ccr_sync.reset();
    return false;
  }
  static_cast<void>(machine.pipeline_continuation(ccr_sync->captured, 1));
  ccr_sync->held_decode = true;
  return true;
}

void DspInstructionPipeline::accept_ccr_decode() {
  const auto word = view.cycle.decode;
  if (!word) { return; }
  const auto memory = decode_memory_stage(word->value);
  const bool bit = memory && memory->peripheral_bit;
  if (!bit && !pipeline_ccr_immediate(word->value)) { return; }
  const auto captured = selected.next;
  if (ccr_sync || !view.cycle.decode_accepted || view.cycle.decode_words != 1 || !captured ||
      captured->word != word) {
    throw Fault("CCR Decode lost its owned one-word F2 occurrence");
  }
  // FM A1 and independent model: bit operations take two nominal periods,
  // ANDI takes three. Following Decode slots serialize actual CCR consumers.
  // Selected reference stages do not establish measured silicon sampling edges.
  ccr_sync = DspPipelineCcrSync{.captured = *captured, .decode_hold = bit ? 1U : 2U};
}

void DspInstructionPipeline::retire_branch() {
  if (branch && branch->word == view.cycle.execute2) { branch_retired = true; }
  release_branch();
}

void DspInstructionPipeline::accept_branch_decode(DspMachine &machine) {
  if (!branch || !branch_target_ready || branch_target_decoded) { return; }
  const auto captured = samples.front();
  if (!captured || captured->word.address != branch->next_pc) {
    throw Fault("Branch target Decode lost its owned F2 occurrence");
  }
  const auto form = decode_branch_form(branch->word.value);
  if (form && form->subroutine_return &&
      machine.pipeline_loop_last(*captured) == captured->word.address) {
    // The unchanged software model adds one period after an RTS target at LA.
    // Serialize the following Decode; do not delay this target's E1/E2 result.
    // This reference placement is not a measured silicon sampling edge.
    branch_decode_hold = 1;
  }
  branch_target_decoded = true;
  release_branch();
}

bool DspInstructionPipeline::capture_omr_branch(DspMachine &machine) {
  if (deferred_omr_branch) {
    const auto captured = *deferred_omr_branch;
    if (!view.cycle.ag2 || view.cycle.ag2 != captured.word || !view.cycle.execute1 ||
        !decode_omr_move(view.cycle.execute1->value) || branch) {
      throw Fault("Deferred OMR branch lost its real predecessor/F2 stages");
    }
    const auto decision =
        machine.capture_reference_branch(captured.word.address, captured.occurrence);
    branch = decision;
    branch_fetch_hold = 0;
    branch_decode_hold = decision.taken ? 0U : 1U;
    branch_target_ready = false;
    branch_target_decoded = false;
    branch_retired = false;
    view.decode_address = decision.next_pc;
    deferred_omr_branch.reset();
    return true;
  }
  const auto ag1 = view.cycle.ag1;
  const auto form = ag1 ? decode_branch_form(ag1->value) : std::nullopt;
  const auto older = view.cycle.ag2;
  if (!form || form->displacement || form->absolute_short || form->address_register ||
      form->absolute_extension || form->relative_extension || form->subroutine_return || !older ||
      !decode_omr_move(older->value)) {
    return false;
  }
  if (branch || loop_setup) { throw Fault("OMR branch barrier has unresolved control flow"); }
  const auto captured = samples.front();
  if (!captured || captured->word != ag1) { throw Fault("OMR branch barrier lost actual F2"); }
  // Resolve this OMR source in AG2 after its older genuine E1 result exists.
  // Target F1 remains in the ordinary AG2 period; no extra instruction stall.
  machine.hold_pipeline_branch(*captured);
  deferred_omr_branch = captured;
  branch_fetch_hold = 1;
  return true;
}

void DspInstructionPipeline::follow_omr_retirement(const DspReferenceCycleResult &bus) {
  if (!bus.progress || !bus.progress->completed || !branch) { return; }
  const auto *effect = std::get_if<DspInstructionRetire>(&*bus.progress->completed);
  if (effect == nullptr || effect->omr == effect->next_omr || branch->omr == effect->next_omr) {
    return;
  }
  const auto form = decode_branch_form(branch->word.value);
  if (branch->omr != effect->omr || !form ||
      (!form->displacement && !form->absolute_short && !form->address_register &&
       !form->absolute_extension && !form->relative_extension && !form->subroutine_return)) {
    throw Fault("Pending branch lacks its actual older OMR execution context");
  }
  branch->omr = effect->next_omr;
}

}
