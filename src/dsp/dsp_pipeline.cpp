// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_pipeline.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_accumulator_transfer.hpp"
#include "dsp/dsp_address_only.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_full_peripheral_store.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pipeline_words.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"

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

bool program_memory_operand(const DspProgramWord word) {
  const auto form = decode_memory_stage(word.value);
  if (!form) { throw Fault("Held memory latency lost its decoded operand"); }
  return is_program_memory(*form);
}

unsigned branch_fetch_delay(const std::optional<DspBranchForm> &form) {
  if (!form) { throw Fault("Native branch lost its qualified fetch form"); }
  // FM A-1: RTS/register JMP/short JSR3, absolute-extension JMP/JSR4,
  // short BSR/BRA4 and long BSR5 retain their selected F1 placement.
  if (form->address_register || form->subroutine_return ||
      (form->absolute_short && form->subroutine_call)) {
    return 0;
  }
  return form->relative_extension ? 2U : 1U;
}

void check_accumulator_context(const DspMachine &machine,
                               const std::optional<DspProgramWord> execute,
                               const bool register_transfer = true) {
  constexpr unsigned scaling_mask = 0xc00;
  const auto cpu = machine.cpu().state();
  if (execute &&
      ((register_transfer && accumulator_transfer_source(execute->value)) ||
       full_accumulator_long_store(execute->value) ||
       full_accumulator_scalar_store(execute->value) ||
       full_accumulator_peripheral_store(execute->value) ||
       partial_accumulator_scalar_store(execute->value) ||
       parallel_partial_tfr_store(execute->value)) &&
      (cpu.sr & scaling_mask) == scaling_mask) {
    throw Fault("Unqualified accumulator MOVE scaling read P:" + hex(execute->address, 6) +
                " pc=" + hex(cpu.pc, 6) + " sr=" + hex(cpu.sr, 6) + " omr=" + hex(cpu.omr, 6));
  }
}

void check_context(const DspMachine &machine, const bool interrupt_deferred) {
  const auto cpu = machine.cpu().state();
  constexpr unsigned unsupported_status = 0xb2000; // CE, SA, FV, SC.
  constexpr unsigned mask_shift = 8;
  constexpr unsigned interrupt_mask = 3;
  if ((cpu.sr & unsupported_status) != 0 ||
      (machine.bus().interrupt_request((cpu.sr >> mask_shift) & interrupt_mask) &&
       !interrupt_deferred)) {
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

void validate_execute_word(const DspProgramFetchView &fetch,
                           const DspPipelineCycle &cycle,
                           const std::optional<DspRepeatControl> result) {
  if (!cycle.execute1) { return; }
  const auto word = *cycle.execute1;
  if (cycle.e1_extension.has_value() != (pipeline_word_count(word) == 2)) {
    throw Fault("Automatic DSP lost its captured extension-stage alignment");
  }
  if (fetch.used != 0 && fetch.words.front() == word && fetch.consume_address == word.address) {
    return;
  }
  if (result && result->remaining == 0 && cycle.execute2 &&
      cycle.execute2->address == result->body && word.address == result->next && fetch.used > 1 &&
      fetch.words[1] == word && fetch.consume_address == result->body) {
    return;
  }
  throw Fault("Automatic DSP E1 lost its actual previously decoded P word");
}

DspFetchedWord decode_receipt(const std::optional<DspFetchedWord> word) {
  if (!word) { throw Fault("Accepted Decode lost its actual F2 receipt"); }
  return *word;
}

bool complete_decode(const std::span<const DspFetchedSample> fetched,
                     const DspFetchedWord opcode,
                     const unsigned words) {
  if (words == 1) { return true; }
  for (std::size_t index = 0; index < fetched.size(); ++index) {
    if (fetched[index] != opcode) { continue; }
    if (index + 1 == fetched.size()) { return false; }
    const auto extension = fetched[index + 1];
    if (extension.sample.address != ((opcode.word.address + 1) & word_mask) ||
        extension.occurrence <= opcode.occurrence) {
      throw Fault("Automatic Decode lost adjacent actual F2 extension read P:" +
                  hex((opcode.word.address + 1) & word_mask, 6) +
                  " pc=" + hex(opcode.word.address, 6));
    }
    return true;
  }
  throw Fault("Automatic Decode lost its actual F2 opcode read P:" + hex(opcode.word.address, 6));
}

struct BranchPoint {
  std::optional<DspProgramWord> word = std::nullopt;
  bool condition = false;
  bool peripheral = false;
};

BranchPoint branch_point(const DspPipelineCycle &cycle) {
  const auto port = cycle.ag2 ? decode_peripheral_read(cycle.ag2->value) : std::nullopt;
  if (port && port->bit) { return {.word = cycle.ag2, .peripheral = true}; }
  if (cycle.ag2 && branch_condition_code(cycle.ag2->value)) {
    return {.word = cycle.ag2, .condition = true};
  }
  if (cycle.ag1 && decode_branch_form(cycle.ag1->value) &&
      !branch_condition_code(cycle.ag1->value)) {
    return {.word = cycle.ag1};
  }
  return {};
}

std::optional<DspProgramWord> extension_tail(const std::optional<DspFetchedInstruction> &phase) {
  if (!phase) { return std::nullopt; }
  if (!phase->extension) { throw Fault("Extension E2 lost its actual F2 word"); }
  return phase->extension->word;
}

DspProgramWord extension_decode_word(const DspInstructionPipeline &pipeline,
                                     const DspFetchedWord opcode,
                                     const std::span<const DspFetchedSample> fetched,
                                     const std::optional<DspProgramWord> ag1,
                                     DspProgramReadContext context) {
  context.pc = opcode.word.address;
  if (ag1 != opcode.word || pipeline.samples.front() != opcode ||
      !complete_decode(fetched, opcode, 2)) {
    throw Fault("Extension Decode lost actual opcode/AG1/F2 ownership pc=" +
                hex(opcode.word.address, 6));
  }
  for (std::size_t index = 0; index + 1 < fetched.size(); ++index) {
    if (fetched[index] == opcode) { return fetched[index + 1].sample.require_known(context); }
  }
  throw Fault("Extension Decode lost its adjacent actual F2 word");
}

void advance_extension_phases(DspInstructionPipeline &pipeline) {
  auto &phases = pipeline.extension_stages;
  for (std::size_t reverse = phases.size() - 1; reverse != 0; --reverse) {
    phases.at(reverse) = phases.at(reverse - 1);
  }
  phases.front().reset();
  if (!pipeline.view.cycle.extension_decode) { return; }
  const auto extension = pipeline.selected.ag1_extension;
  if (!pipeline.decoding_extension || !extension ||
      extension->word != pipeline.view.cycle.extension_decode) {
    throw Fault("Extension phase lost its actual opcode/F2 pair");
  }
  phases.front() =
      DspFetchedInstruction{.opcode = *pipeline.decoding_extension, .extension = extension};
}

void prepare_decode(DspInstructionPipeline &pipeline,
                    const DspMachine &machine,
                    const DspProgramFetchView &fetch,
                    const std::span<const DspFetchedSample> fetched,
                    DspPipelineCycle &cycle) {
  if (pipeline.held_program_memory || pipeline.hold_ccr_decode(machine) ||
      pipeline.hold_repeat_decode()) {
    return;
  }
  if (pipeline.decoding_extension) {
    cycle.extension_decode =
        extension_decode_word(pipeline,
                              *pipeline.decoding_extension,
                              fetched,
                              cycle.ag1,
                              {.pc = machine.cpu().state().pc, .omr = fetch.omr});
    return;
  }
  if (pipeline.hold_loop_decode() || pipeline.deferred_omr_branch) { return; }
  pipeline.held_branch_decode =
      pipeline.branch && pipeline.branch_target_ready && pipeline.branch_decode_hold != 0;
  if (pipeline.branch && (!pipeline.branch_target_ready || pipeline.held_branch_decode)) { return; }
  // Even an already prefetched successor cannot enter Decode while AG1 resolves
  // this branch and cuts the discarded path from the owned queue.
  if (const auto ag1 = cycle.ag1; ag1 && decode_branch_form(ag1->value)) { return; }
  // CCR control resolves in AG2, after the older genuine E1 flag candidate.
  if (cycle.ag2 && branch_condition_code(cycle.ag2->value)) { return; }
  for (const auto word : {cycle.ag1, cycle.ag2}) {
    const auto read = word ? decode_peripheral_read(word->value) : std::nullopt;
    if (read && read->bit) { return; }
  }
  // Select the FIFO position after the actual older E1/AG2/AG1 occurrences.
  // Decode sees the prior cycle's F2 latch, never the new F2 of this clock.
  if (pipeline.selected.next_sample) {
    const auto word = pipeline.selected.next_sample->sample.require_known(
        {.pc = machine.cpu().state().pc, .omr = fetch.omr});
    if (!pipeline.selected.next || pipeline.selected.next->word != word) {
      throw Fault("Known Decode lost its owned actual F2 value");
    }
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

void DspInstructionPipeline::select_words(const std::span<const DspFetchedSample> fetched,
                                          const std::uint32_t omr) {
  const auto repeated = repeat_setup ? repeat_setup->body : std::nullopt;
  selected = select_pipeline_samples(fetched, samples, extension_samples, repeated, {.omr = omr});
  if (repeat_setup && repeated && repeat_setup->remaining != 0) {
    selected.next = repeated;
    selected.next_sample = DspFetchedSample::retain(*repeated);
  }
  if (repeated) {
    const auto word = repeated->word;
    if (!fetch_repeat_body(word.value)) {
      throw Fault("Unqualified repeated Decode read P:" + hex(word.address, 6) + " opcode=" +
                  hex(word.value, 6) + " pc=" + hex(word.address, 6) + " omr=" + hex(omr, 6));
    }
  }
}

void DspInstructionPipeline::prepare(const DspMachine &machine) {
  check_context(machine, machine.pipeline_interrupt_deferred());
  check_accumulator_context(machine, view.stages[2]);
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
  cycle.extension_execute2 = extension_tail(extension_stages.back());
  std::array<DspFetchedSample, dsp_program_fetch_capacity> fetched{};
  for (std::size_t index = 0; index < fetch.used; ++index) {
    const auto sample = machine.pipeline_fetched_sample(index);
    if (!sample) { throw Fault("Automatic DSP lost its actual F2 receipt"); }
    fetched.at(index) = *sample;
  }
  validate_stage_samples(view, samples);
  select_words(std::span(fetched).first(fetch.used), fetch.omr);
  prepare_program_memory(machine, cycle.execute1);
  prepare_decode(*this, machine, fetch, std::span(fetched).first(fetch.used), cycle);
  check_accumulator_context(machine, cycle.decode, false);
  validate_program_decode(machine, cycle);
  if (selected.ag1_extension) { cycle.ag1_extension = selected.ag1_extension->word; }
  cycle.e1_extension = extensions[1];
  validate_execute_word(fetch, cycle, machine.pipeline_repeat_result());
  const std::array words{cycle.ag2, cycle.execute1};
  for (std::size_t index = 0; index < words.size(); ++index) {
    const auto word = words.at(index);
    const bool memory = profile == DspFetchPipeline::automatic_memory && word &&
                        (decode_memory_stage(word->value).has_value() ||
                         decode_address_only(word->value).has_value());
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
      cycle.memory_source_stall = memory_source_dependency(inputs) != 0 ||
                                  pipeline_loop_source_dependency(inputs) ||
                                  accumulator_transfer_dependency(inputs) != 0;
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
  if (awaiting_bit_write) {
    const auto *bit = std::get_if<DspNativeAccess>(&pending->effect);
    if (bit == nullptr || bit->pc != view.awaiting_retirement->address) {
      throw Fault("RMW E2 lost its actual pending bit callback");
    }
    // Native-owned capture validation belongs to the machine bus preflight.
    return pending->ticket;
  }
  const auto *effect = std::get_if<DspInstructionRetire>(&pending->effect);
  if (effect == nullptr || effect->pc != view.awaiting_retirement->address ||
      (effect->next_omr != effect->omr && !decode_omr_move(view.awaiting_retirement->value))) {
    throw Fault("Automatic DSP E2 differs from its actual native candidate");
  }
  const auto port_read = decode_peripheral_read(view.awaiting_retirement->value);
  if (decode_branch_form(view.awaiting_retirement->value) || (port_read && port_read->bit)) {
    validate_branch(*effect);
  } else if (!machine.pipeline_loop_result() && !machine.pipeline_repeat_result() &&
             (effect->redirected ||
              effect->next_pc !=
                  ((effect->pc + pipeline_word_count(*view.awaiting_retirement)) & word_mask))) {
    throw Fault("Automatic DSP E2 differs from its actual sequential ALU candidate");
  }
  const auto inputs = view.awaiting_inputs;
  if (!inputs) { throw Fault("Automatic DSP E2 lost its functional input record"); }
  const auto visible = awaiting_visible_accumulators;
  if (!visible) { throw Fault("Automatic DSP E2 lost its actual E1 accumulator visibility"); }
  const auto actual = machine.cpu().state();
  const auto deferred = view.awaiting_deferred_accumulators;
  if (((deferred & 1U) != 0 && actual.a != visible->at(0)) ||
      ((deferred & 2U) != 0 && actual.b != visible->at(1))) {
    throw Fault("Automatic DSP E2 forwarded operand differs from its actual native candidate");
  }
  return pending->ticket;
}

std::optional<DspProgramWord> DspInstructionPipeline::decoded_word(const DspMachine &machine) {
  if (view.cycle.extension_decode) {
    decoding_extension.reset();
    return std::nullopt;
  }
  if (!view.cycle.decode || view.cycle.agu_dependencies != 0) { return std::nullopt; }
  const auto opcode = decode_receipt(selected.next);
  if (decoding_extension) { throw Fault("Opcode Decode replaced an owned extension phase"); }
  if (view.cycle.decode_words == 1) { return view.cycle.decode; }
  const auto fetch = machine.reference_fetch();
  std::array<DspFetchedSample, dsp_program_fetch_capacity> fetched{};
  for (std::size_t index = 0; index < fetch.used; ++index) {
    const auto sample = machine.pipeline_fetched_sample(index);
    if (!sample) { throw Fault("Automatic Decode lost its actual F2 receipt"); }
    fetched.at(index) = *sample;
  }
  if (!complete_decode(std::span(fetched).first(fetch.used), opcode, view.cycle.decode_words)) {
    return std::nullopt;
  }
  // Table2-1: opcode AG1 overlaps the actual extension's following Decode.
  decoding_extension = opcode;
  return view.cycle.decode;
}

namespace {
void capture_accumulator_visibility(DspInstructionPipeline &pipeline, const DspMachine &machine) {
  const auto &cycle = pipeline.view.cycle;
  const auto inputs = pipeline.view.awaiting_inputs;
  if (!inputs) { throw Fault("Actual E1 lost its functional accumulator operands"); }
  const auto move = cycle.memory ? cycle.memory->move : cycle.transfer;
  const auto fields = move ? move->fields : DspMoveFields{};
  const auto actual = machine.cpu().state();
  // A real E1 MOVE can overwrite an operand forwarded from the preceding ALU.
  // Keep its published visibility distinct from the native functional inputs.
  pipeline.awaiting_visible_accumulators =
      std::array{fields.a != 0 ? actual.a : inputs->a, fields.b != 0 ? actual.b : inputs->b};
}

template <typename Result>
void capture_addressed_result(DspInstructionPipeline &pipeline,
                              const Result &result,
                              const bool bit_read = false) {
  const auto word = pipeline.view.cycle.execute1;
  if (!word || !result.progress.pending || result.address.word != *word) {
    throw Fault("Automatic addressed E1 lost its actual native result");
  }
  pipeline.awaiting_bit_write = bit_read;
  pipeline.view.cycle.candidate_inputs = result.inputs;
  pipeline.view.cycle.execution = result.progress;
  pipeline.view.cycle.execution_captured = true;
  pipeline.view.awaiting_retirement = word;
  pipeline.view.awaiting_inputs = result.inputs;
  pipeline.view.awaiting_deferred_accumulators = pipeline.view.cycle.e1_deferred_accumulators;
}
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
    follow_omr_retirement(bus);
    view.cycle.retirement_published = true;
    retire_branch();
    retire_repeat(machine);
    awaiting_bit_write = false;
    view.awaiting_retirement.reset();
    view.awaiting_inputs.reset();
    view.awaiting_deferred_accumulators = 0;
    awaiting_visible_accumulators.reset();
  }
  if (view.cycle.memory_source_stall || held_program_memory) { return; }
  if (view.cycle.execute1) {
    if (bus.memory) {
      view.cycle.memory = bus.memory;
      capture_addressed_result(*this, *bus.memory, bus.memory->bit_read);
    } else if (bus.address_only) {
      view.cycle.address_only = bus.address_only;
      capture_addressed_result(*this, *bus.address_only);
    } else {
      execute(machine);
    }
    capture_accumulator_visibility(*this, machine);
  }
  complete_program_memory();
  capture_loop_target(machine);
  capture_repeat_target(machine);
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
  advance_extension_phases(*this);
  view.stages[2] = view.stages[1];
  view.stages[1] = view.stages[0];
  view.stages[0] = decoded_word(machine);
  if (view.stages[0]) {
    samples[0] = selected.next;
    view.decode_address =
        machine.pipeline_continuation(decode_receipt(selected.next), view.cycle.decode_words);
    view.cycle.decode_accepted = true;
    // Confirm this target before it opens its own loop/repeat barrier.
    accept_branch_decode(machine);
    accept_loop_decode(machine);
    accept_repeat_decode();
    accept_ccr_decode();
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
    if (decode_memory_stage(instruction.value) || decode_address_only(instruction.value)) {
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
  if (profile != DspFetchPipeline::automatic_memory || capture_omr_branch(machine)) { return; }
  const auto ag1 = view.cycle.ag1;
  const auto early = ag1 ? decode_peripheral_read(ag1->value) : std::nullopt;
  if (early && early->bit) {
    // FM A-1/NXP windows: relative bit branches add one target-fetch period.
    branch_fetch_hold = early->relative ? 2U : 1U;
  }
  const auto point = branch_point(view.cycle);
  if (!point.word) { return; }
  const auto word = *point.word;
  if (branch) { throw Fault("Automatic branch cannot replace an unresolved fetch decision"); }
  DspBranchDecision decision;
  if (point.peripheral) {
    const auto ticket = reads[0];
    if (!ticket) { throw Fault("Peripheral AG2 branch lost its owned AG1 response"); }
    decision = machine.preview_pipeline_peripheral_branch(*ticket);
  } else {
    const auto captured = samples[point.condition ? 1U : 0U];
    if (!captured || captured->word != word) {
      throw Fault("Branch stage lost its actual F2 occurrence");
    }
    decision = machine.capture_reference_branch(word.address, captured->occurrence);
    // Moving CCR resolution from AG1 to AG2 consumes the short-branch F1 hold.
    branch_fetch_hold = point.condition ? 0U : branch_fetch_delay(decode_branch_form(word.value));
  }
  branch = decision;
  branch_target_decoded = false;
  branch_retired = false;
  view.decode_address = decision.next_pc;
  // Register control resolves in AG1; CCR/port control follows older E1 in AG2.
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
  if (!branch || !branch_target_decoded || !branch_retired || branch_decode_hold != 0) { return; }
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
    // Independent windows distinguish the waits: indexed/predecrement AGU
    // latency overlaps source readiness; P-memory latency follows readiness.
    if (view.cycle.memory_source_stall && program_memory_operand(program_memory->captured.word)) {
      return;
    }
    --program_memory->remaining;
    return;
  }
  if (ccr_sync && ccr_sync->held_decode && ccr_sync->decode_hold != 0) { --ccr_sync->decode_hold; }
  if (loop_setup && loop_setup->held_decode && loop_setup->decode_hold != 0) {
    --loop_setup->decode_hold;
  }
  // A native REP count-source wait extends setup; its held E1 cannot spend
  // one of the nominal Decode-to-body setup periods (retained NXP windows).
  const bool count_source_held = repeat_setup && view.cycle.memory_source_stall &&
                                 view.cycle.execute1 == repeat_setup->prefix.word;
  if (repeat_setup && repeat_setup->held_decode && repeat_setup->setup_hold != 0 &&
      !count_source_held) {
    --repeat_setup->setup_hold;
  }
  if (branch_fetch_hold != 0) { --branch_fetch_hold; }
  if (held_branch_decode && branch_decode_hold != 0) { --branch_decode_hold; }
  release_branch();
}
}
