// SPDX-License-Identifier: GPL-3.0-only
// Directed native controller activation under the explicit reference-cycle order.
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

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned source_register = 0xffffef;
constexpr unsigned destination_register = 0xffffee;
constexpr unsigned counter_register = 0xffffed;
constexpr unsigned control_register = 0xffffec;
constexpr unsigned source = 0x660;
constexpr unsigned destination = 0x900;
constexpr unsigned literal = 0xa5c39f;
constexpr unsigned untouched = 0x123456;
constexpr unsigned software_copy = 0x9802d4;
constexpr unsigned enable = 0x800000;
constexpr unsigned cycles = 24;
constexpr unsigned padding = 64;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
unsigned assertions = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw Fault(message); }
}

void run() {
  DspMachine machine;
  test::DspProgram program;
  for (const auto store : {DspStore{.address = source_register, .value = source},
                           DspStore{.address = destination_register, .value = destination},
                           DspStore{.address = counter_register},
                           DspStore{.address = control_register, .value = software_copy}}) {
    program.emit("movep #>" + test::number(store.value) + ",x:<<" + test::number(store.address));
  }
  for (unsigned index = 0; index < padding; ++index) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = source, .value = literal});
  machine.bus().write({.space = DspSpace::y, .address = destination, .value = untouched});
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
  unsigned controls = 0;
  unsigned sources = 0;
  unsigned destinations = 0;
  std::optional<std::uint64_t> enabled;
  for (unsigned index = 0; index < cycles; ++index) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline) { throw Fault("The real automatic pipeline must own each CPU cycle"); }
    ++assertions;
    if (const auto memory = point.pipeline->memory) {
      const auto target = memory->address.data_accesses[0];
      require(target.target.space == DspSpace::x && target.operation == DspMemoryOperation::write,
              "Native MOVEP owns the actual controller store footprint");
      ++controls;
      if (target.target.address == control_register) {
        enabled = point.completed_cycles;
        require(!point.bus.dma.phase, "DE becomes visible after this cycle's prior DMA phase");
        require(
            (machine.bus().read({.space = DspSpace::x, .address = control_register}) & enable) != 0,
            "The genuine native DCR store enables the channel");
      }
    }
    if (const auto phase = point.bus.dma.phase) {
      if (!enabled) { throw Fault("No DMA phase may precede the real enable store"); }
      ++assertions;
      require(phase->channel == 0 && phase->source.address == source &&
                  phase->source.space == DspDmaSpace::x &&
                  phase->destination.address == destination &&
                  phase->destination.space == DspDmaSpace::y,
              "The controller uses the literal original source and destination preloads");
      if (phase->kind == DspDmaPhaseKind::source_read) {
        ++sources;
        require(point.completed_cycles == *enabled + 1 && !point.bus.dma.word,
                "The next real reference cycle captures source data without early destination");
        require(machine.bus().read({.space = DspSpace::y, .address = destination}) == untouched,
                "Source capture cannot prematurely modify destination memory");
      } else {
        ++destinations;
        require(point.completed_cycles == *enabled + 2 && point.bus.dma.word &&
                    point.bus.dma.word->value == literal,
                "A distinct following cycle writes the actual captured literal word");
      }
    }
  }
  require(controls == 4 && sources == 1 && destinations == 1,
          "Each controller store and source/destination phase occurs exactly once");
  require(machine.bus().read({.space = DspSpace::y, .address = destination}) == literal &&
              (machine.bus().read({.space = DspSpace::x, .address = control_register}) & enable) ==
                  0,
          "The actual copy completes and the one-shot channel clears DE");
  std::cout << "controls=" << controls << " sources=" << sources << " destinations=" << destinations
            << " assertions=" << assertions << '\n';
}
}

int main() {
  try {
    run();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
