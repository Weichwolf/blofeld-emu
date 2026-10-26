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
#include "dsp/dsp_pipeline_words.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
constexpr auto execution = DspReferenceExecution::staged_register_boundaries;

void check_context(const DspMachine &machine) {
  const auto cpu = machine.cpu().state();
  constexpr unsigned unsupported_status = 0xb2000; // CE, SA, FV, SC.
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

void validate_stage_samples(const DspPipelineView &view,
                            const std::array<std::optional<DspFetchedWord>, 3> &samples) {
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto sample = samples.at(index);
    const auto word = view.stages.at(index);
    if (sample && word && sample->word == *word) { continue; }
    if (!sample && !word) { continue; }
    throw Fault("Automatic DSP lost its stage/F2 occurrence alignment");
  }
}

void validate_execute_word(const DspProgramFetchView &fetch, const DspPipelineCycle &cycle) {
  if (!cycle.execute1) { return; }
  const auto word = *cycle.execute1;
  if (cycle.e1_extension.has_value() != (pipeline_word_count(word) == 2)) {
    throw Fault("Automatic DSP lost its captured extension-stage alignment");
  }
  if (fetch.used == 0 || fetch.words.front() != word || fetch.consume_address != word.address) {
    throw Fault("Automatic DSP E1 lost its actual previously decoded P word");
  }
}

DspFetchedWord decode_receipt(const std::optional<DspFetchedWord> word) {
  if (!word) { throw Fault("Accepted Decode lost its actual F2 receipt"); }
  return *word;
}

void prepare_decode(DspInstructionPipeline &pipeline,
                    const DspMachine &machine,
                    const DspProgramFetchView &fetch,
                    DspPipelineCycle &cycle) {
  if (pipeline.held_program_memory || pipeline.hold_ccr_decode(machine)) { return; }
  if (pipeline.hold_loop_decode()) { return; }
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
  // Select the FIFO position after the actual older E1/AG2/AG1 occurrences.
  // Decode sees the prior cycle's F2 latch, never the new F2 of this clock.
  if (pipeline.selected.next) {
    const auto word = pipeline.selected.next->word;
    if (word.address != pipeline.view.decode_address) {
      throw Fault("Automatic Decode cursor differs from its actual F2 occurrence pc=" +
                  hex(word.address, 6) + " omr=" + hex(fetch.omr, 6));
    }
    cycle.decode = word;
    cycle.decode_words = decode_pipeline_word(word, fetch.omr, pipeline.profile).words;
    cycle.agu_dependencies = pipeline_agu_dependencies(pipeline.view.stages, word);
  }
}
}

void DspInstructionPipeline::prepare(const DspMachine &machine) {
  check_context(machine);
  constexpr unsigned loop_flag = 0x8000;
  if ((machine.cpu().state().sr & loop_flag) != 0 && !machine.pipeline_loop_control()) {
    throw Fault("Automatic DSP LF requires its observed native/F2 owner pc=" +
                hex(machine.cpu().state().pc, 6));
  }
  const auto fetch = machine.reference_fetch();
  DspPipelineCycle cycle{.ag1 = view.stages[0],
                         .ag2 = view.stages[1],
                         .execute1 = view.stages[2],
                         .execute2 = view.awaiting_retirement};
  std::array<DspFetchedWord, dsp_program_fetch_capacity> fetched{};
  for (std::size_t index = 0; index < fetch.used; ++index) {
    const auto sample = machine.pipeline_fetched_word(index);
    if (!sample) { throw Fault("Automatic DSP lost its actual F2 receipt"); }
    fetched.at(index) = *sample;
  }
  validate_stage_samples(view, samples);
  selected =
      select_pipeline_words(std::span(fetched).first(fetch.used), samples, extension_samples);
  prepare_program_memory(machine, cycle.execute1);
  prepare_decode(*this, machine, fetch, cycle);
  validate_program_decode(machine, cycle);
  if (selected.ag1_extension) { cycle.ag1_extension = selected.ag1_extension->word; }
  cycle.e1_extension = extensions[1];
  validate_execute_word(fetch, cycle);
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
      const DspInstructionDependency inputs{.producer = *cycle.execute2,
                                            .consumer = *cycle.execute1};
      cycle.memory_source_stall =
          memory_source_dependency(inputs) != 0 || pipeline_loop_source_dependency(inputs);
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
  } else if (!machine.pipeline_loop_result() &&
             (effect->redirected ||
              effect->next_pc !=
                  ((effect->pc + pipeline_word_count(*view.awaiting_retirement)) & word_mask))) {
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
  capture_loop_f2(bus);
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
    retire_branch();
    view.awaiting_retirement.reset();
    view.awaiting_inputs.reset();
    view.awaiting_deferred_accumulators = 0;
  }
  if (view.cycle.memory_source_stall || held_program_memory) { return; }
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
  complete_program_memory();
  capture_loop_target(machine);
  view.cycle.ag2_address = bus.address_update;
  capture_branch(machine);
  capture_address(machine);
  reads[1] = reads[0];
  reads[0] = bus.peripheral_read;
  extension_samples[1] = extension_samples[0];
  extension_samples[0] = selected.ag1_extension;
  extensions[1] = extensions[0];
  extensions[0] = view.cycle.ag1_extension;
  samples[2] = samples[1];
  samples[1] = samples[0];
  samples[0].reset();
  view.stages[2] = view.stages[1];
  view.stages[1] = view.stages[0];
  view.stages[0] = view.cycle.agu_dependencies != 0 ? std::nullopt : view.cycle.decode;
  if (view.stages[0]) {
    samples[0] = selected.next;
    view.decode_address =
        machine.pipeline_continuation(decode_receipt(selected.next), view.cycle.decode_words);
    view.cycle.decode_accepted = true;
    accept_loop_decode(machine);
    accept_ccr_decode();
    accept_branch_decode();
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
      const auto captured = samples[0];
      next = machine.capture_reference_address(instruction.address,
                                               captured ? captured->occurrence : 0);
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
    const auto captured = samples[0];
    if (!captured || captured->word != *ag1) {
      throw Fault("Branch AG1 lost its actual F2 occurrence");
    }
    decision = machine.capture_reference_branch(ag1->address, captured->occurrence);
    const auto form = decode_branch_form(ag1->value);
    branch_fetch_hold = form && form->address_register ? 0U : 1U;
  }
  branch = decision;
  branch_target_decoded = false;
  branch_retired = false;
  view.decode_address = decision.next_pc;
  // Register control resolves in AG1; port sources follow older E1 writes in AG2.
  // Direct Rn issues target F1 in the resolving AG1 period; the older forms
  // retain their later F1 and the FM false Decode delay.
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

void DspInstructionPipeline::release_branch() {
  if (!branch || !branch_target_decoded || !branch_retired) { return; }
  branch.reset();
  branch_decode_hold = 0;
  branch_target_ready = false;
  branch_target_decoded = false;
  branch_retired = false;
}

std::uint64_t DspInstructionPipeline::peripheral_occurrence() const {
  const auto ag1 = view.cycle.ag1;
  if (!ag1 || !decode_peripheral_read(ag1->value)) { return 0; }
  const auto captured = samples[0];
  if (!captured || captured->word != *ag1) {
    throw Fault("Peripheral AG1 lost its actual F2 occurrence");
  }
  return captured->occurrence;
}

void DspInstructionPipeline::finish_cycle() {
  if (held_program_memory) {
    if (!program_memory || program_memory->remaining == 0) {
      throw Fault("Held program-memory execution lost its actual latency owner");
    }
    --program_memory->remaining;
    return;
  }
  if (ccr_sync && ccr_sync->held_decode && ccr_sync->decode_hold != 0) { --ccr_sync->decode_hold; }
  if (loop_setup && loop_setup->held_decode && loop_setup->decode_hold != 0) {
    --loop_setup->decode_hold;
  }
  if (branch_fetch_hold != 0) { --branch_fetch_hold; }
  if (held_branch_decode && branch_decode_hold != 0) { --branch_decode_hold; }
}
}
