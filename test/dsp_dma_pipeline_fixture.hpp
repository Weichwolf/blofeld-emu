// SPDX-License-Identifier: GPL-3.0-only
#pragma once

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

#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::dma_pipeline {
using namespace blofeld;
inline constexpr unsigned source_register = 0xffffef;
inline constexpr unsigned destination_register = 0xffffee;
inline constexpr unsigned counter_register = 0xffffed;
inline constexpr unsigned control_register = 0xffffec;
inline constexpr unsigned source = 0x660;
inline constexpr unsigned ram_entry = 0x100;
inline constexpr unsigned long_jump = 0x0af080;
inline constexpr unsigned destination = 0x900;
inline constexpr unsigned first = 0xa5c39f;
inline constexpr unsigned second = 0x123456;
inline constexpr unsigned untouched = 0x654321;
inline constexpr unsigned software_copy = 0x9802d4;
inline constexpr unsigned word_copy = 0x880244;
inline constexpr unsigned enable = 0x800000;
inline constexpr unsigned maximum_cycles = 128;
inline constexpr unsigned extal = 12000000;
inline constexpr unsigned timebase = 540000000;
inline unsigned assertions = 0;
inline unsigned refusals = 0;
inline unsigned attempted_advances = 0;
inline unsigned completed_advances = 0;

inline void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw Fault(message); }
}

inline std::string reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const Fault &error) {
    ++refusals;
    return error.what();
  }
  throw Fault(message);
}

struct Rig {
  DspMachine machine;
  std::optional<DspFetchRun> clock;
  unsigned enable_pc = 0;
  std::vector<unsigned> body_pc;

  Rig(const std::vector<std::string> &body,
      const unsigned control = software_copy,
      const unsigned count = 0,
      const unsigned target = destination,
      const bool enter_ram = false) {
    test::DspProgram program;
    program.immediate("r0", source);
    program.immediate("r1", destination);
    program.immediate("x0", second);
    for (const auto store : {DspStore{.address = source_register, .value = source},
                             DspStore{.address = destination_register, .value = target},
                             DspStore{.address = counter_register, .value = count},
                             DspStore{.address = control_register, .value = control}}) {
      if (store.address == control_register) { enable_pc = program.pc(); }
      program.emit("movep #>" + test::number(store.value) + ",x:<<" + test::number(store.address));
    }
    if (enter_ram) {
      // Independent FM long JMP encoding; no host program-counter assignment.
      program.word(long_jump);
      program.word(ram_entry);
    }
    for (const auto &text : body) {
      body_pc.push_back(program.pc());
      program.emit(text);
    }
    for (unsigned index = 0; index < maximum_cycles; ++index) { program.emit("nop"); }
    program.install(machine.bus());
    machine.bus().write({.space = DspSpace::x, .address = source, .value = first});
    for (unsigned index = 0; index < 4; ++index) {
      machine.bus().write(
          {.space = DspSpace::y, .address = destination + index, .value = untouched});
    }
    machine.bus().pll().observe_pinit(false);
    machine.cpu().reset_spi_boot();
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    clock.emplace(DspFetchRunInputs{.extal_hz = extal,
                                    .timebase_hz = timebase,
                                    .timing = DspFetchTiming::reference_cycle_order,
                                    .execution = DspReferenceExecution::staged_register_boundaries,
                                    .pipeline = DspFetchPipeline::automatic_memory},
                  machine);
  }

  DspFetchCycleResult tick(const DspDmaGrant grant = DspDmaGrant::available,
                           const unsigned requests = 0,
                           const unsigned known = 15) {
    ++attempted_advances;
    auto result = run().advance_cycle(
        machine, {.known_requests = known, .requests = requests, .grant = grant});
    ++completed_advances;
    return result;
  }

  unsigned memory(const DspSpace space, const unsigned address) {
    return machine.bus().read({.space = space, .address = address});
  }

  [[nodiscard]] DspFetchRun &run() {
    if (!clock) { throw Fault("Own DMA fixture has no constructed clock"); }
    return *clock;
  }

  [[nodiscard]] const DspFetchRun &run() const {
    if (!clock) { throw Fault("Own DMA fixture has no constructed clock"); }
    return *clock;
  }

  [[nodiscard]] DspPipelineView pipeline() const {
    const auto state = run().pipeline();
    if (!state) { throw Fault("Own DMA fixture lost its automatic pipeline"); }
    return *state;
  }

  [[nodiscard]] bool approaching(const unsigned pc) const {
    const auto state = pipeline();
    const auto word = state.stages[2];
    return word && word->address == pc;
  }

  void until_enable() {
    for (unsigned budget = 0; budget < maximum_cycles; ++budget) {
      const auto point = tick();
      if (point.bus.memory && point.bus.memory->data.target.address == control_register) {
        require(point.bus.memory->data.pc == enable_pc &&
                    (memory(DspSpace::x, control_register) & enable) != 0,
                "The automatic native E1 write, not a host register assignment, enables DMA");
        return;
      }
    }
    throw Fault("Own controller setup did not reach its bounded native enable write");
  }
};

void source_conflict();
void captured_source();
void destination_hold_restore();
void disable_captured_word();
void controller_guards();
void request_knowledge_restore();
}
