// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_loop_publication.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_loop_source.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned control_mask = 0xffff00;
constexpr unsigned unsupported_forecast_status = 0xa2000; // CE, SA, SC; execution remains native.
constexpr unsigned word_mask = 0xffffff;

DspLoopArchitecture control(const dsp56300::Registers &registers) {
  return {.pc = registers.pc,
          .omr = registers.omr,
          .la = registers.la,
          .lc = registers.lc,
          .sp = registers.sp,
          .sc = registers.sc,
          .status = registers.sr & control_mask};
}

bool unpredicted_loop_control(const std::uint32_t opcode) {
  const auto shape = dsp56300::inspect_instruction(opcode);
  if (!shape) { return true; }
  using enum dsp56300::InstructionKind;
  static constexpr std::array controls{jump_short,
                                       jump_effective,
                                       call_short,
                                       call_effective,
                                       return_subroutine,
                                       return_interrupt,
                                       and_control,
                                       or_control,
                                       loop_immediate,
                                       loop_register,
                                       loop_short,
                                       loop_effective,
                                       loop_forever,
                                       break_loop,
                                       end_loop,
                                       conditional_jump,
                                       relative_branch,
                                       move_control_effective,
                                       move_control_short,
                                       move_control_register,
                                       move_control_immediate,
                                       bit_register,
                                       bit_jump,
                                       bit_branch};
  return std::ranges::contains(controls, shape->kind);
}
} // namespace

bool native_loop_publication(const DspFetchedInstruction *admitted,
                             const dsp56300::PendingInstruction &pending) {
  if (admitted == nullptr || pending.effect.kind != dsp56300::EffectKind::pcu_update ||
      !automatic_loop_source(admitted->opcode.word.value)) {
    return false;
  }
  if (!admitted->extension || pending.effect.pc != admitted->opcode.word.address ||
      pending.opcode != admitted->opcode.word.value || !pending.effect.pcu_update ||
      pending.effect.pcu_update->opcode != admitted->opcode.word.value) {
    throw Fault("Native loop publication lost its captured opcode read P:" +
                hex(admitted->opcode.word.address, 6) + " pc=" + hex(pending.effect.pc, 6) +
                " omr=" + hex(pending.effect.omr, 6));
  }
  return true;
}

void observe_loop_publication(DspProgramFetch &fetch,
                              const DspFetchedInstruction &admitted,
                              const dsp56300::Registers &before,
                              const dsp56300::Registers &after) {
  admitted.check_consumed(fetch.consumed_pair(), before.omr);
  if (before.pc != admitted.opcode.word.address || !admitted.extension || before.omr != after.omr) {
    throw Fault("Native loop publication changed its owned context read P:" +
                hex(admitted.opcode.word.address, 6) + " pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
  if ((before.sr & (loop_flag | unsupported_forecast_status)) != 0) {
    // A nested setup remains native. Its actual retirement discards the outer
    // forecast through the consumed-control witness, not a generic redirect.
    return;
  }
  const auto body = (admitted.opcode.word.address + 2) & word_mask;
  const bool relative = relative_loop(admitted.opcode.word.value);
  const auto last =
      relative ? (admitted.opcode.word.address + admitted.extension->word.value) & word_mask
               : admitted.extension->word.value;
  if (body > last ||
      (relative && (admitted.opcode.word.address > word_mask - 2 ||
                    admitted.extension->word.value > word_mask - admitted.opcode.word.address))) {
    return; // Wrapped extents keep the conservative native refill policy.
  }
  fetch.open_loop({.word = admitted.opcode.word,
                   .extension = admitted.extension->word,
                   .before = control(before),
                   .after = control(after)},
                  admitted.opcode,
                  *admitted.extension);
}

bool confirm_loop_publication(DspProgramFetch &fetch,
                              const DspFetchedInstruction &admitted,
                              const dsp56300::Registers &registers,
                              const dsp56300::InstructionFetch &context,
                              const dsp56300::InstructionOutcome &outcome) {
  const auto actual = control(registers);
  if (fetch.confirm_opening(actual, outcome.control_transfer)) { return true; }
  if (unpredicted_loop_control(outcome.opcode)) {
    return !context.latched_opcode &&
           fetch.retire_native_control(admitted,
                                       {.pc = outcome.address,
                                        .next_pc = context.address,
                                        .omr = registers.omr,
                                        .redirected = outcome.control_transfer});
  }
  return fetch.confirm_loop(actual, outcome.control_transfer);
}

void redirect_loop_control(DspProgramFetch &fetch,
                           const DspFetchedInstruction &admitted,
                           const DspFetchCommit actual) {
  if (!fetch.retire_native_control(admitted, actual)) {
    fetch.redirect(actual.next_pc, actual.omr);
  }
}
} // namespace blofeld
