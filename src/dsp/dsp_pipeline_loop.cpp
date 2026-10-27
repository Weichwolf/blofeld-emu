// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"

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

bool DspInstructionPipeline::hold_repeat_decode() {
  if (!repeat_setup) { return false; }
  auto &setup = *repeat_setup;
  setup.held_decode = !setup.body || setup.setup_hold != 0;
  return setup.held_decode;
}

void DspInstructionPipeline::accept_repeat_decode() {
  const auto captured = samples.front();
  if (!captured) { return; }
  if (fetch_repeat_prefix(captured->word.value)) {
    if (repeat_setup || loop_setup || branch) {
      throw Fault("Unqualified overlapping automatic REP setup read P:" +
                  hex(captured->word.address, 6));
    }
    repeat_setup = DspPipelineRepeat{.prefix = *captured, .body = std::nullopt};
    return;
  }
  if (repeat_setup && repeat_setup->body == captured) {
    auto &setup = *repeat_setup;
    if (setup.remaining == 0 || setup.setup_hold != 0) {
      throw Fault("Repeated Decode escaped its native count/setup owner");
    }
    --setup.remaining;
    view.decode_address =
        setup.remaining == 0 ? captured->word.address + 1 : captured->word.address;
  }
}

void DspInstructionPipeline::capture_repeat_target(const DspMachine &machine) {
  if (!repeat_setup || view.cycle.execute1 != repeat_setup->prefix.word) { return; }
  const auto result = machine.pipeline_repeat_result();
  const auto body = machine.pipeline_repeat_body();
  if (!result || !body || result->remaining == 0 || result->loaded ||
      result->body != body->word.address ||
      body->word.address != repeat_setup->prefix.word.address + 1) {
    throw Fault("REP E1 lost its actual native result/body F2 receipt");
  }
  repeat_setup->body = body;
  repeat_setup->remaining = result->remaining;
}

void DspInstructionPipeline::retire_repeat(const DspMachine &machine) {
  if (!repeat_setup || !repeat_setup->body || view.cycle.execute2 != repeat_setup->body->word ||
      machine.pipeline_repeat_control()) {
    return;
  }
  if (repeat_setup->remaining != 0) {
    throw Fault("Native REP completed before all owned logical Decode iterations");
  }
  repeat_setup.reset();
}

}
