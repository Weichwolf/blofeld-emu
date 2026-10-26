// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_pipeline.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <optional>
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

void prepare_decode(DspInstructionPipeline &pipeline,
                    const DspProgramFetchView &fetch,
                    DspPipelineCycle &cycle) {
  pipeline.held_branch_decode =
      pipeline.branch && pipeline.branch_target_ready && pipeline.branch_decode_hold != 0;
  if (pipeline.branch && (!pipeline.branch_target_ready || pipeline.held_branch_decode)) { return; }
  // Even an already prefetched successor cannot enter Decode while AG1 resolves
  // this branch and cuts the discarded path from the owned queue.
  if (const auto ag1 = cycle.ag1; ag1 && decode_branch_form(ag1->value)) { return; }
  for (const auto word : {cycle.ag1, cycle.ag2}) {
    const auto read = word ? decode_peripheral_read(word->value) : std::nullopt;
    if (read && read->bit) { return; }
  }
  // Decode sees the prior cycle's F2 latch, never the new F2 of this clock.
  for (std::size_t index = 0; index < fetch.used; ++index) {
    if (fetch.words.at(index).address == pipeline.view.decode_address) {
      cycle.decode = fetch.words.at(index);
      const auto word = *cycle.decode;
      cycle.decode_words = decode_pipeline_word(word, fetch.omr, pipeline.profile).words;
      cycle.agu_dependencies = pipeline_agu_dependencies(pipeline.view.stages, word);
      return;
    }
  }
}
}

void DspInstructionPipeline::prepare(const DspMachine &machine) {
  check_context(machine);
  const auto fetch = machine.reference_fetch();
  DspPipelineCycle cycle{.ag1 = view.stages[0],
                         .ag2 = view.stages[1],
                         .execute1 = view.stages[2],
                         .execute2 = view.awaiting_retirement};
  prepare_decode(*this, fetch, cycle);
  if (cycle.ag1) { cycle.ag1_extension = pipeline_extension(fetch, *cycle.ag1); }
  cycle.e1_extension = extensions[1];
  if (cycle.execute1 &&
      cycle.e1_extension.has_value() != (pipeline_word_count(*cycle.execute1) == 2)) {
    throw Fault("Automatic DSP lost its captured extension-stage alignment");
  }
  if (cycle.execute1 && (fetch.used == 0 || fetch.words.front() != *cycle.execute1 ||
                         fetch.consume_address != cycle.execute1->address)) {
    throw Fault("Automatic DSP E1 lost its actual previously decoded P word");
  }
  const std::array words{cycle.ag2, cycle.execute1};
  for (std::size_t index = 0; index < words.size(); ++index) {
    const auto word = words.at(index);
    const bool memory = profile == DspFetchPipeline::automatic_memory && word &&
                        decode_memory_stage(word->value).has_value();
    const bool port_read = profile == DspFetchPipeline::automatic_memory && word &&
                           decode_peripheral_read(word->value).has_value();
    if (reads.at(index).has_value() != port_read) {
      throw Fault("Automatic DSP lost its owned peripheral-read stage alignment");
    }
    const auto ticket = addresses.at(index);
    if (ticket.has_value() != memory) {
      throw Fault("Automatic DSP lost its address-stage alignment");
    }
    if (ticket) {
      const auto address = machine.address_stage(*ticket);
      if (address.word != word || address.published != (index == 1)) {
        throw Fault("Automatic DSP address continuation differs from its actual stage word");
      }
    }
  }
  if (cycle.execute1) {
    cycle.e1_operands = machine.cpu().state();
    if (cycle.execute2) {
      cycle.e1_deferred_accumulators = deferred_alu_accumulators(*cycle.execute2, *cycle.execute1);
      cycle.memory_source_stall =
          memory_source_dependency({.producer = *cycle.execute2, .consumer = *cycle.execute1}) != 0;
    }
  }
  if (cycle.execute2) {
    cycle.e2_operands = machine.cpu().state();
    cycle.e2_deferred_accumulators = view.awaiting_deferred_accumulators;
  }
  view.cycle = cycle;
}

DspReferenceTicket DspInstructionPipeline::retirement(const DspMachine &machine) const {
  const auto pending = machine.reference_pending();
  if (!pending || !view.awaiting_retirement || pending->execution != execution) {
    throw Fault("Automatic DSP E2 lost its actual native candidate");
  }
  const auto *effect = std::get_if<DspInstructionRetire>(&pending->effect);
  if (effect == nullptr || effect->pc != view.awaiting_retirement->address ||
      effect->next_omr != effect->omr) {
    throw Fault("Automatic DSP E2 differs from its actual native candidate");
  }
  const auto port_read = decode_peripheral_read(view.awaiting_retirement->value);
  if (decode_branch_form(view.awaiting_retirement->value) || (port_read && port_read->bit)) {
    validate_branch(*effect);
  } else if (effect->redirected ||
             effect->next_pc !=
                 ((effect->pc + pipeline_word_count(*view.awaiting_retirement)) & word_mask)) {
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

void DspInstructionPipeline::complete(DspMachine &machine, const DspReferenceCycleResult &bus) {
  if (bus.dma.core_stall) { return; }
  if (branch && !branch_target_ready && bus.fetch) {
    const auto &fetch = *bus.fetch;
    if (fetch.used == 0 || fetch.words.at(fetch.used - 1).address != branch->next_pc) {
      throw Fault("Automatic branch lost its fresh target F2 pc=" + hex(branch->word.address, 6) +
                  " omr=" + hex(branch->omr, 6));
    }
    branch_target_ready = true;
  }
  if (view.cycle.execute2) {
    if (!bus.progress || !bus.progress->retired) {
      throw Fault("Automatic DSP E2 did not publish its selected actual native result");
    }
    view.cycle.retirement_published = true;
    view.awaiting_retirement.reset();
    view.awaiting_inputs.reset();
    view.awaiting_deferred_accumulators = 0;
  }
  if (view.cycle.memory_source_stall) { return; }
  if (view.cycle.execute1) {
    if (bus.memory) {
      if (!bus.memory->progress.pending || bus.memory->address.word != *view.cycle.execute1) {
        throw Fault("Automatic memory E1 lost its actual native result");
      }
      view.cycle.memory = bus.memory;
      view.cycle.candidate_inputs = bus.memory->inputs;
      view.cycle.execution = bus.memory->progress;
      view.cycle.execution_captured = true;
      view.awaiting_retirement = view.cycle.execute1;
      view.awaiting_inputs = bus.memory->inputs;
      view.awaiting_deferred_accumulators = view.cycle.e1_deferred_accumulators;
    } else {
      execute(machine);
    }
  }
  view.cycle.ag2_address = bus.address_update;
  capture_branch(machine);
  capture_address(machine);
  reads[1] = reads[0];
  reads[0] = bus.peripheral_read;
  extensions[1] = extensions[0];
  extensions[0] = view.cycle.ag1_extension;
  view.stages[2] = view.stages[1];
  view.stages[1] = view.stages[0];
  view.stages[0] = view.cycle.agu_dependencies != 0 ? std::nullopt : view.cycle.decode;
  if (view.stages[0]) {
    view.decode_address = (view.decode_address + view.cycle.decode_words) & word_mask;
    view.cycle.decode_accepted = true;
    if (branch && branch_target_ready) {
      branch.reset();
      branch_decode_hold = 0;
      branch_target_ready = false;
    }
  }
  view.cycle.advanced = true;
}

void DspInstructionPipeline::rebind(const DspMachine &machine) {
  for (auto &ticket : reads) {
    if (ticket) { ticket = machine.rebind_reference_peripheral_read(*ticket); }
  }
  for (auto &ticket : addresses) {
    if (ticket) { ticket = machine.rebind_address_stage(*ticket); }
  }
}

void DspInstructionPipeline::capture_address(DspMachine &machine) {
  std::optional<DspAddressStageTicket> next;
  const auto word = view.cycle.ag1;
  if (profile == DspFetchPipeline::automatic_memory && word) {
    const auto instruction = *word;
    if (decode_memory_stage(instruction.value)) {
      next = machine.capture_reference_address(instruction.address);
      view.cycle.ag1_address = machine.address_stage(*next);
    }
  }
  addresses[1] = addresses[0];
  addresses[0] = next;
}

void DspInstructionPipeline::capture_branch(DspMachine &machine) {
  if (profile != DspFetchPipeline::automatic_memory) { return; }
  const auto ag1 = view.cycle.ag1;
  const auto early = ag1 ? decode_peripheral_read(ag1->value) : std::nullopt;
  if (early && early->bit) { branch_fetch_hold = 1; }
  const auto ag2 = view.cycle.ag2;
  const auto read = ag2 ? decode_peripheral_read(ag2->value) : std::nullopt;
  const bool peripheral = read && read->bit;
  const bool omr = ag1 && decode_branch_form(ag1->value).has_value();
  if (!peripheral && !omr) { return; }
  if (branch) { throw Fault("Automatic branch cannot replace an unresolved fetch decision"); }
  DspBranchDecision decision;
  if (peripheral) {
    const auto ticket = reads[0];
    if (!ticket) { throw Fault("Peripheral AG2 branch lost its owned AG1 response"); }
    decision = machine.preview_pipeline_peripheral_branch(*ticket);
  } else {
    decision = machine.capture_reference_branch(ag1->address);
    branch_fetch_hold = 1;
  }
  branch = decision;
  view.decode_address = decision.next_pc;
  // OMR resolves in AG1; the port source resolves after older E1 writes in AG2.
  // Both issue the selected F1 after AG2 and retain the FM false Decode delay.
  branch_decode_hold = decision.taken ? 0U : 1U;
  branch_target_ready = false;
}

void DspInstructionPipeline::validate_branch(const DspInstructionRetire &retirement) const {
  if (!branch || retirement.pc != branch->word.address || retirement.next_pc != branch->next_pc ||
      retirement.omr != branch->omr || retirement.next_omr != branch->omr ||
      retirement.redirected != branch->taken) {
    throw Fault(
        "Automatic branch differs from its actual native result pc=" + hex(retirement.pc, 6) +
        " next=" + hex(retirement.next_pc, 6) + " omr=" + hex(retirement.omr, 6));
  }
}

void DspInstructionPipeline::finish_cycle() {
  if (branch_fetch_hold != 0) { --branch_fetch_hold; }
  if (held_branch_decode && branch_decode_hold != 0) { --branch_decode_hold; }
}
}
