// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>

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
  if (!word || !pipeline_ccr_immediate(word->value)) { return; }
  const auto captured = selected.next;
  if (ccr_sync || !view.cycle.decode_accepted || view.cycle.decode_words != 1 || !captured ||
      captured->word != word) {
    throw Fault("CCR Decode lost its owned one-word F2 occurrence");
  }
  // FM A1 and unchanged model: three nominal periods. The two following
  // Decode slots serialize CCR consumers while genuine prefetch continues.
  // Selected reference stages do not establish measured silicon sampling edges.
  ccr_sync = DspPipelineCcrSync{.captured = *captured};
}

void DspInstructionPipeline::retire_branch() {
  if (branch && branch->word == view.cycle.execute2) { branch_retired = true; }
  release_branch();
}

void DspInstructionPipeline::accept_branch_decode() {
  if (!branch || !branch_target_ready) { return; }
  branch_target_decoded = true;
  release_branch();
}

}
