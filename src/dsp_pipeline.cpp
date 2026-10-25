// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_pipeline.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_pipeline.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <cstddef>
#include <variant>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

void check_context(const DspMachine &machine) {
  const auto cpu = machine.cpu().state();
  constexpr unsigned unsupported_status = 0xba000; // CE, SA, FV, LF, SC.
  constexpr unsigned mask_shift = 8;
  constexpr unsigned interrupt_mask = 3;
  if ((cpu.sr & unsupported_status) != 0 ||
      machine.bus().interrupt_request((cpu.sr >> mask_shift) & interrupt_mask)) {
    throw Fault("Unqualified automatic DSP mode/interrupt pc=" + hex(cpu.pc, 6) +
                " sr=" + hex(cpu.sr, 6) + " omr=" + hex(cpu.omr, 6));
  }
  if (machine.reference_retirement()) {
    throw Fault("Automatic DSP stages cannot adopt an explicit overlap candidate");
  }
}
}

void DspAluPipeline::prepare(const DspMachine &machine) {
  check_context(machine);
  const auto fetch = machine.reference_fetch();
  DspPipelineCycle cycle{.ag1 = view.stages[0],
                         .ag2 = view.stages[1],
                         .execute1 = view.stages[2],
                         .execute2 = view.awaiting_retirement};
  // Decode sees the prior cycle's F2 latch, never the new F2 of this clock.
  for (std::size_t index = 0; index < fetch.used; ++index) {
    if (fetch.words.at(index).address == view.decode_address) {
      cycle.decode = fetch.words.at(index);
      validate_alu_pipeline_word(*cycle.decode, fetch.omr);
      break;
    }
  }
  if (cycle.execute1 && (fetch.used == 0 || fetch.words.front() != *cycle.execute1 ||
                         fetch.consume_address != cycle.execute1->address)) {
    throw Fault("Automatic DSP E1 lost its actual previously decoded P word");
  }
  if (cycle.execute1) {
    cycle.e1_operands = machine.cpu().state();
    if (cycle.execute2) {
      cycle.e1_deferred_accumulators = deferred_alu_accumulators(*cycle.execute2, *cycle.execute1);
    }
  }
  if (cycle.execute2) {
    cycle.e2_operands = machine.cpu().state();
    cycle.e2_deferred_accumulators = view.awaiting_deferred_accumulators;
  }
  view.cycle = cycle;
}

DspReferenceTicket DspAluPipeline::retirement(const DspMachine &machine) const {
  const auto pending = machine.reference_pending();
  if (!pending || !view.awaiting_retirement || pending->execution != execution) {
    throw Fault("Automatic DSP E2 lost its actual native candidate");
  }
  const auto *effect = std::get_if<DspInstructionRetire>(&pending->effect);
  if (effect == nullptr || effect->pc != view.awaiting_retirement->address || effect->redirected ||
      effect->next_pc != ((effect->pc + 1) & word_mask) || effect->next_omr != effect->omr) {
    throw Fault("Automatic DSP E2 differs from its actual sequential ALU candidate");
  }
  const auto inputs = view.awaiting_inputs;
  if (!inputs) { throw Fault("Automatic DSP E2 lost its functional input record"); }
  const auto actual = machine.cpu().state();
  const auto deferred = view.awaiting_deferred_accumulators;
  if (((deferred & 1U) != 0 && actual.a != inputs->a) ||
      ((deferred & 2U) != 0 && actual.b != inputs->b)) {
    throw Fault("Automatic DSP E2 forwarded operand differs from its actual native candidate");
  }
  return pending->ticket;
}

void DspAluPipeline::execute(DspMachine &machine) {
  if (!view.cycle.execute1) { throw Fault("Automatic DSP E1 lost its phase word"); }
  view.cycle.candidate_inputs = machine.cpu().state();
  auto pending = machine.reference_pending();
  if (!pending) {
    const auto progress = machine.begin_reference_instruction(execution);
    pending = progress.pending;
  }
  if (!pending) { throw Fault("Automatic DSP E1 lost its actual opcode boundary"); }
  const auto *access = std::get_if<DspNativeAccess>(&pending->effect);
  if (access == nullptr || access->kind != DspNativeAccessKind::opcode ||
      access->target.address != view.cycle.execute1->address ||
      pending->latched_program_word != view.cycle.execute1->value) {
    throw Fault("Automatic DSP E1 opcode does not match the real decoded F2 word");
  }
  const auto progress = machine.advance_reference_access(pending->ticket);
  if (!progress.pending || progress.retired ||
      !std::holds_alternative<DspInstructionRetire>(progress.pending->effect)) {
    throw Fault("Unqualified actual native effect in automatic DSP ALU E1");
  }
  view.cycle.execution = progress;
  view.cycle.execution_captured = true;
  view.awaiting_retirement = view.cycle.execute1;
  view.awaiting_inputs = view.cycle.candidate_inputs;
  view.awaiting_deferred_accumulators = view.cycle.e1_deferred_accumulators;
}

void DspAluPipeline::complete(DspMachine &machine, const DspReferenceCycleResult &bus) {
  if (bus.dma.core_stall) { return; }
  if (view.cycle.execute2) {
    if (!bus.progress || !bus.progress->retired) {
      throw Fault("Automatic DSP E2 did not publish its selected actual native result");
    }
    view.cycle.retirement_published = true;
    view.awaiting_retirement.reset();
    view.awaiting_inputs.reset();
    view.awaiting_deferred_accumulators = 0;
  }
  if (view.cycle.execute1) { execute(machine); }
  view.stages[2] = view.stages[1];
  view.stages[1] = view.stages[0];
  view.stages[0] = view.cycle.decode;
  if (view.cycle.decode) { view.decode_address = (view.decode_address + 1) & word_mask; }
  view.cycle.advanced = true;
}
}
