// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
DspFetchOrigin DspProgramFetch::origin(const DspFetchedWord captured) const {
  if (captured.occurrence == 0) { throw Fault("F2 origin requires an actual occurrence"); }
  for (std::size_t index = 0; index < consumed_.size(); ++index) {
    if (consumed_.at(index) == captured) { return consumed_origins_.at(index); }
  }
  return origins_.at(position(captured));
}

std::uint32_t DspProgramFetch::execution_omr(const DspFetchedWord captured) const {
  static_cast<void>(position(captured));
  if (!context_) { return state_.omr; }
  const auto last = context_->writer.extension.value_or(context_->writer.opcode);
  return captured.occurrence > last.occurrence ? context_->after : context_->before;
}

void DspProgramFetch::validate_context_storage(const DspBus &bus, const std::uint32_t omr) const {
  for (std::size_t index = 0; index < state_.used; ++index) {
    const auto address = state_.words.at(index).address;
    if (!bus.stable_program_context(address, omr) ||
        origins_.at(index).resource != bus.memory_resource({.target = {.address = address}})) {
      throw Fault("Unqualified OMR transition of captured read P:" + hex(address, 6) +
                  " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(omr, 6));
    }
  }
  if (state_.pending_address && !bus.stable_program_context(*state_.pending_address, omr)) {
    throw Fault("Unqualified OMR transition of issued read P:" + hex(*state_.pending_address, 6) +
                " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(omr, 6));
  }
}

void DspProgramFetch::stage_context(const DspInstructionRetire &result, const DspBus &bus) {
  constexpr unsigned mask = 0xffffff;
  constexpr unsigned supported = 0x60038f; // MSW/MS, priority, reset mode field.
  const auto opcode =
      consumed_.back().word.address == result.pc ? consumed_.back() : consumed_.front();
  const auto form = decode_omr_move(opcode.word.value);
  if (!state_.active || context_ || loop_ || opening_ || consumed_end_ || !form ||
      opcode.word.address != result.pc || result.omr != state_.omr || result.redirected ||
      result.next_pc != ((result.pc + form->words) & mask) ||
      result.next_pc != state_.consume_address ||
      ((result.omr ^ result.next_omr) & ~supported) != 0) {
    throw Fault("Unqualified native OMR context pc=" + hex(result.pc, 6) +
                " omr=" + hex(result.omr, 6) + " next=" + hex(result.next_omr, 6));
  }
  const DspFetchedInstruction writer{.opcode = opcode,
                                     .extension = form->words == 2 ? std::optional(consumed_.back())
                                                                   : std::nullopt};
  writer.check_consumed(consumed_, result.omr);
  validate_context_storage(bus, result.next_omr);
  if (branch_ && result.omr != result.next_omr) {
    const auto branch = decode_branch_form(branch_->word.value);
    const auto last = writer.extension.value_or(writer.opcode);
    if (!branch || (!branch->displacement && !branch->address_register) ||
        branch_occurrence_ <= last.occurrence || branch_->omr != result.omr) {
      throw Fault("Unqualified OMR source of pending branch pc=" + hex(result.pc, 6));
    }
  }
  context_ = DspFetchContext{.writer = writer, .before = result.omr, .after = result.next_omr};
  if (branch_) { branch_->omr = result.next_omr; }
}

bool DspProgramFetch::confirm_context(const DspFetchCommit completed, const DspBus &bus) {
  const auto pc = completed.pc;
  const auto next_pc = completed.next_pc;
  const auto omr = completed.omr;
  const auto redirected = completed.redirected;
  if (!context_ || context_->writer.opcode.word.address != pc) { return false; }
  const auto &control = *context_;
  const auto last = control.writer.extension.value_or(control.writer.opcode);
  constexpr unsigned mask = 0xffffff;
  if (control.before != state_.omr || control.after != omr || redirected ||
      next_pc != ((last.word.address + 1) & mask) || next_pc != state_.consume_address) {
    throw Fault("Native OMR retirement differs from actual F2 context pc=" + hex(pc, 6));
  }
  control.writer.check_consumed(consumed_, control.before);
  validate_context_storage(bus, omr);
  state_.omr = omr;
  context_.reset();
  return true;
}
}
