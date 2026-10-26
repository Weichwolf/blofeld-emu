// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <cstddef>
#include <optional>
#include <variant>

namespace blofeld {
bool DspInstructionPipeline::hold_loop_decode() {
  if (!loop_setup) { return false; }
  auto &setup = *loop_setup;
  setup.held_decode = setup.target_ready && setup.decode_hold != 0;
  return !setup.target_ready || setup.held_decode;
}

void DspInstructionPipeline::accept_loop_decode(DspMachine &machine) {
  if (loop_setup) {
    if (!loop_setup->target_ready || loop_setup->decode_hold != 0) {
      throw Fault("DO target Decode escaped its actual F2/setup barrier");
    }
    loop_setup.reset();
  }
  const auto captured = samples.front();
  if (!captured || !pipeline_counted_loop(captured->word)) { return; }
  if (branch) { throw Fault("DO Decode cannot replace an unresolved branch"); }
  // Older words may still precede the actual decoded DO. Find its adjacent
  // extension by exact occurrence, never by the first equal program address.
  std::optional<DspFetchedWord> extension;
  for (std::size_t index = 0; index + 1 < machine.reference_fetch().used; ++index) {
    if (machine.pipeline_fetched_word(index) == captured) {
      extension = machine.pipeline_fetched_word(index + 1);
      break;
    }
  }
  if (!extension || extension->word.address != captured->word.address + 1) {
    throw Fault("DO Decode lost its actual adjacent F2 extension");
  }
  machine.hold_pipeline_loop(*captured);
  loop_setup = DspPipelineLoopSetup{.captured = {.opcode = *captured, .extension = extension},
                                    .target = std::nullopt};
}

void DspInstructionPipeline::capture_loop_target(DspMachine &machine) {
  const auto word = view.cycle.execute1;
  if (!word || !pipeline_counted_loop(*word)) { return; }
  if (!loop_setup || loop_setup->captured.opcode.word != *word || !view.cycle.execution ||
      !view.cycle.execution->pending || !machine.pipeline_loop_result()) {
    throw Fault("Automatic DO E1 lost its actual native-owned F2 opening");
  }
  const auto *effect = std::get_if<DspInstructionRetire>(&view.cycle.execution->pending->effect);
  if (effect == nullptr || effect->pc != word->address || effect->next_omr != effect->omr) {
    throw Fault("Automatic DO E1 lost its actual held native control result");
  }
  auto &setup = *loop_setup;
  setup.target = effect->next_pc;
  setup.decode_hold = effect->redirected ? 3U : 0U;
  view.decode_address = effect->next_pc;
}

void DspInstructionPipeline::capture_loop_f2(const DspReferenceCycleResult &bus) {
  if (!loop_setup || !loop_setup->target || loop_setup->target_ready || !bus.fetch) { return; }
  auto &setup = *loop_setup;
  const auto &fetch = *bus.fetch;
  if (fetch.used == 0 || fetch.words.at(fetch.used - 1).address != *setup.target ||
      !setup.captured.extension) {
    throw Fault("Automatic DO lost its actual selected target F2");
  }
  setup.target_ready = true;
  // FM A2.6.5: positive one-instruction bodies add one Decode period. Derive
  // size from the actual first F2 opcode; zero-count annulment adds three.
  const auto target = fetch.words.at(fetch.used - 1);
  if (setup.decode_hold == 0 &&
      target.address + pipeline_word_count(target) - 1 == setup.captured.extension->word.value) {
    setup.decode_hold = 1;
  }
}
}
