// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned limit = 32;
constexpr unsigned literal = 0x123456;
constexpr unsigned single_minimum = 7;
constexpr unsigned double_minimum = 8;

void trace(const DspFetchCycleResult &point, const unsigned words) {
  if (!point.pipeline) { throw Fault("Own phase control lost its actual pipeline"); }
  const auto pipeline = *point.pipeline;
  std::cout << "words=" << words << " completed=" << point.completed_cycles
            << " F1=" << (point.issued_address ? hex(*point.issued_address, 6) : "none")
            << " F2=" << (point.bus.fetch ? std::to_string(point.bus.fetch->used) : "none")
            << " decode=" << (pipeline.decode ? hex(pipeline.decode->address, 6) : "none")
            << " E1=" << (pipeline.execute1 ? hex(pipeline.execute1->address, 6) : "none")
            << " E2=" << (pipeline.execute2 ? hex(pipeline.execute2->address, 6) : "none")
            << " extension-Decode="
            << (pipeline.extension_decode ? hex(pipeline.extension_decode->address, 6) : "none")
            << " extension-E2="
            << (pipeline.extension_execute2 ? hex(pipeline.extension_execute2->address, 6) : "none")
            << '\n';
}

void replay(DspMachine &machine,
            DspFetchRun &clock,
            const DspFetchCheckpoint &saved,
            const DspDebugState &expected) {
  clock.restore(saved, machine);
  for (unsigned period = 5; period <= double_minimum; ++period) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (point.completed_cycles != period) { throw Fault("Joint word-phase restore lost time"); }
    if (period != double_minimum) { continue; }
    if (!point.pipeline || !point.pipeline->advanced ||
        point.pipeline->extension_execute2 !=
            DspProgramWord{.address = test::boot + 1, .value = literal} ||
        machine.cpu().debug_state() != expected) {
      throw Fault("Joint restore lost the actual extension tail or repeated native effects");
    }
  }
}

void check_extension_decode(const DspPipelineCycle &pipeline,
                            const std::uint64_t completed,
                            const bool extended) {
  if (!extended || completed != 4 || !pipeline.ag1 || pipeline.ag1->address != test::boot ||
      pipeline.extension_decode != DspProgramWord{.address = test::boot + 1, .value = literal} ||
      pipeline.ag1_extension != pipeline.extension_decode || pipeline.decode_accepted) {
    throw Fault("Table2-1 requires actual extension Decode alongside opcode AG1 in period4");
  }
}

void check_completion(const DspMachine &machine,
                      const DspFetchCycleResult &point,
                      const bool extended,
                      const unsigned words,
                      const bool native_retired) {
  if (!point.pipeline) { throw Fault("Own phase completion lost its actual pipeline"); }
  const auto pipeline = *point.pipeline;
  if (!native_retired) { throw Fault("A real extension E2 cannot replace native execution"); }
  if (extended &&
      pipeline.extension_execute2 != DspProgramWord{.address = test::boot + 1, .value = literal}) {
    throw Fault("Extension E2 lost its independently specified actual program word");
  }
  if (extended &&
      machine.cpu().state().x != (static_cast<std::uint64_t>(literal) << test::word_bits)) {
    throw Fault("The own native immediate value did not retire correctly");
  }
  const auto minimum = extended ? double_minimum : single_minimum;
  if (point.completed_cycles != minimum) {
    throw Fault("FM2.3.1/Table2-1 initial latency differs:words=" + std::to_string(words) +
                " actual=" + std::to_string(point.completed_cycles) +
                " expected=" + std::to_string(minimum));
  }
}

void one(const bool extended) {
  DspMachine machine;
  test::DspProgram program;
  if (extended) {
    program.immediate("x1", literal);
  } else {
    program.emit("nop");
  }
  const auto words = program.pc() - test::boot;
  for (unsigned i = 0; i < limit; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun clock({.extal_hz = extal,
                     .timebase_hz = timebase,
                     .timing = DspFetchTiming::reference_cycle_order,
                     .execution = DspReferenceExecution::staged_register_boundaries,
                     .pipeline = DspFetchPipeline::automatic_memory},
                    machine);
  bool native_retired = false;
  std::optional<DspFetchCheckpoint> saved;
  for (unsigned cycle = 0; cycle < limit; ++cycle) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline) { throw Fault("Own phase control lost its actual pipeline"); }
    const auto pipeline = *point.pipeline;
    trace(point, words);
    if (pipeline.extension_decode) {
      check_extension_decode(pipeline, point.completed_cycles, extended);
      saved = clock.checkpoint(machine);
    }
    native_retired |= pipeline.retirement_published && pipeline.execute2 &&
                      pipeline.execute2->address == test::boot;
    const bool complete =
        extended ? pipeline.advanced && pipeline.extension_execute2.has_value() : native_retired;
    if (!complete) { continue; }
    check_completion(machine, point, extended, words, native_retired);
    if (extended) {
      if (!saved) {
        throw Fault("The actual extension Decode never supplied its joint checkpoint");
      }
      replay(machine, clock, *saved, machine.cpu().debug_state());
    }
    return;
  }
  throw Fault("Own initial word pipeline did not complete within its explicit phase bound");
}
}

int main() {
  try {
    one(false);
    one(true);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
