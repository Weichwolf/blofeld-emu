// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <optional>

namespace blofeld {
namespace {
bool program_word(const std::optional<DspProgramWord> word) {
  const auto form = word ? decode_memory_stage(word->value) : std::nullopt;
  return form && is_program_transfer(*form);
}
}

void DspInstructionPipeline::validate_program_decode(const DspMachine &machine,
                                                     const DspPipelineCycle &cycle) const {
  if (!program_word(cycle.decode)) { return; }
  const auto captured = selected.next;
  if (!captured || captured->word != cycle.decode) {
    throw Fault("Program-memory Decode lost its actual F2 receipt");
  }
  const auto last = machine.pipeline_loop_last(*captured);
  if (last == captured->word.address) {
    throw Fault(
        "Unqualified program transfer at owned loop end read P:" + hex(captured->word.address, 6) +
        " pc=" + hex(captured->word.address, 6) + " omr=" + hex(machine.cpu().state().omr, 6));
  }
}

void DspInstructionPipeline::prepare_program_memory(const DspMachine &machine,
                                                    const std::optional<DspProgramWord> word) {
  held_program_memory = false;
  if (!program_word(word)) {
    if (program_memory) { throw Fault("Program-memory latency escaped its actual E1 word"); }
    return;
  }
  const auto captured = samples[2];
  if (!captured || captured->word != word) {
    throw Fault("Program-memory E1 lost its actual decoded F2 receipt");
  }
  if (!program_memory) {
    if (branch || loop_setup) {
      throw Fault("Unqualified unresolved control during program-memory E1 read P:" +
                  hex(captured->word.address, 6) + " pc=" + hex(captured->word.address, 6) +
                  " omr=" + hex(machine.cpu().state().omr, 6));
    }
    // FM A1 and unchanged NXP windows: six total periods, including the first
    // retirement's extra five. Existing AGU hazards remain independent. This
    // selected E1 hold does not establish a measured silicon bus-sample edge.
    program_memory = DspPipelineProgramMemory{.captured = *captured};
  }
  if (program_memory->captured != *captured) {
    throw Fault("Program-memory latency changed its owned instruction occurrence");
  }
  held_program_memory = program_memory->remaining != 0;
}

void DspInstructionPipeline::complete_program_memory() {
  if (!program_memory) { return; }
  const auto memory = view.cycle.memory;
  if (held_program_memory || program_memory->remaining != 0 || !memory ||
      memory->address.word != program_memory->captured.word ||
      memory->data.kind != DspNativeAccessKind::read || !memory->additional_data ||
      memory->additional_data->kind != DspNativeAccessKind::write ||
      memory->additional_data->target.space != DspSpace::p) {
    throw Fault("Program-memory E1 did not commit its actual native peripheral/P pair");
  }
  program_memory.reset();
}
}
