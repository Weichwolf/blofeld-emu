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

#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned profile = 0x600389;
constexpr unsigned start = 0x660;
constexpr unsigned value = 0x123456;
constexpr unsigned first_output = 0x234500;
constexpr unsigned gdb = 0xfffffc;
constexpr unsigned canary = 0x654321;
constexpr unsigned limit = 256;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
unsigned assertions = 0;
unsigned extension_periods = 0;
unsigned suffixes = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw Fault(message); }
}

struct Progress {
  unsigned bodies = 0;
  unsigned outputs = 0;
};

struct Stream {
  unsigned repeat = 0;
  unsigned outputs = 0;
};

struct Rig {
  DspMachine machine;
  std::optional<DspFetchRun> clock;
  unsigned body = 0;
  std::vector<unsigned> outputs;

  explicit Rig(const Stream input) {
    const auto repeat = input.repeat;
    const auto count = input.outputs;
    test::DspProgram program;
    program.immediate("x1", profile);
    program.emit("move x1,omr");
    for (unsigned i = 0; i < 8; ++i) { program.emit("nop"); }
    program.immediate("r4", start);
    program.immediate("a", value);
    if (repeat != 0) {
      program.immediate("x0", repeat);
      program.emit("rep x0");
    }
    body = program.pc();
    program.emit("move a,l:(r4)+");
    for (unsigned i = 0; i < count; ++i) {
      outputs.push_back(program.pc());
      program.emit("movep #>" + test::number(first_output + i) + ",x:<<" + test::number(gdb));
    }
    for (unsigned i = 0; i < limit; ++i) { program.emit("nop"); }
    program.install(machine.bus());
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      machine.bus().write({.space = space, .address = start - 1, .value = canary});
      machine.bus().write(
          {.space = space, .address = start + (repeat == 0 ? 1 : repeat), .value = canary});
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

  DspFetchRun &run() {
    if (!clock) { throw Fault("Missing own instruction-fetch clock"); }
    return *clock;
  }

  bool tick(Progress &progress) {
    const auto point =
        run().advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline) { throw Fault("Actual automatic pipeline must remain present"); }
    const auto pipeline = *point.pipeline;
    if (pipeline.retirement_published && pipeline.execute2 && pipeline.execute2->address == body) {
      ++progress.bodies;
    }
    if (point.bus.memory && point.bus.memory->data.target.address == gdb) {
      require(progress.outputs < outputs.size(), "Native output cannot duplicate a MOVEP");
      require(point.bus.memory->data.pc == outputs.at(progress.outputs) &&
                  point.bus.memory->data.value == first_output + progress.outputs &&
                  machine.bus().once_latch() == first_output + progress.outputs,
              "Actual E1 output must preserve its independent instruction and literal");
      ++progress.outputs;
    }
    const bool held = pipeline.advanced && pipeline.extension_decode.has_value();
    if (held) {
      require(
          pipeline.extension_decode && pipeline.ag1 &&
              pipeline.ag1_extension == pipeline.extension_decode &&
              pipeline.extension_decode->address == pipeline.ag1->address + 1 &&
              !pipeline.decode_accepted,
          "Table2-1 extension Decode overlaps its actual opcode AG1 and excludes younger Decode");
    }
    if (held) { ++extension_periods; }
    return held;
  }

  void finish(Progress &progress, const unsigned repeat) {
    for (unsigned i = 0; i < limit && progress.outputs < outputs.size(); ++i) {
      static_cast<void>(tick(progress));
    }
    const unsigned bodies = repeat == 0 ? 1 : repeat;
    require(progress.outputs == outputs.size() && progress.bodies == bodies &&
                machine.cpu().state().r[4] == start + bodies,
            "All independent native REP bodies and consecutive outputs must complete exactly");
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      require(machine.bus().read({.space = space, .address = start - 1}) == canary &&
                  machine.bus().read({.space = space, .address = start + bodies}) == canary,
              "The native body cannot write outside its literal bounds");
      for (unsigned i = 0; i < bodies; ++i) {
        require(machine.bus().read({.space = space, .address = start + i}) ==
                    (space == DspSpace::x ? value : 0),
                "Long stores must retain the independently specified X/Y halves");
      }
    }
  }
};

void restore_extension() {
  Rig rig({.repeat = 3, .outputs = 16});
  Progress progress;
  for (unsigned i = 0; i < limit; ++i) {
    const auto before = rig.run().checkpoint(rig.machine);
    const auto saved = progress;
    if (!rig.tick(progress)) { continue; }
    const auto after = rig.run().checkpoint(rig.machine);
    const auto held = progress;
    rig.finish(progress, 3);
    for (const bool preceding : {true, false}) {
      rig.run().restore(preceding ? before : after, rig.machine);
      progress = preceding ? saved : held;
      rig.finish(progress, 3);
      ++suffixes;
    }
    return;
  }
  throw Fault("Own consecutive MOVEP stream did not expose an extension fetch hold");
}
}

int main() {
  try {
    for (const unsigned repeat : {0U, 1U, 3U, 17U}) {
      for (const unsigned count : {1U, 4U, 8U, 16U}) {
        Rig rig({.repeat = repeat, .outputs = count});
        Progress progress;
        rig.finish(progress, repeat);
      }
    }
    restore_extension();
    require(extension_periods != 0 && suffixes == 2,
            "Actual hold and both checkpoint suffixes must execute");
    std::cout << "streams=17 restored-suffixes=" << suffixes
              << " extension-decode-periods=" << extension_periods << " assertions=" << assertions
              << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
