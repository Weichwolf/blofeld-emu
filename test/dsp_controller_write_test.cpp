// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_native_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {
using namespace blofeld;
namespace fixture = blofeld::test::fetch_native;
using blofeld::test::esai::require;
constexpr unsigned first = 0xffffd8, last = 0xfffff3, status = 0xfffff4;
constexpr unsigned source0 = 0xffffef, control0 = 0xffffec, peripheral_high = 0xffffc0;
constexpr unsigned movep_immediate = 0x08f480, software = 0x9802d4, enable = 0x800000;
constexpr unsigned source = 0x800, destination = 0x900;
constexpr unsigned payload = 0x123456, canary = 0x654321, written = 0x345;
constexpr unsigned period = 45, instruction_bound = 16;
enum class Mode : std::uint8_t { quiet, held, committed };

struct Case {
  unsigned address;
  Mode mode;
  OwnDspAutomaticTiming timing;
};

DspDmaInputs inputs(OwnDspMachine &machine, const DspDmaGrant grant) {
  const auto last_cycle = machine.bus().last_dma_cycle();
  return {.cycle = last_cycle ? *last_cycle + 1 : 0,
          .known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = grant};
}

unsigned value(const unsigned address) {
  if (address <= source0 && (source0 - address) % 4 == 3) {
    return address == control0 ? software & ~enable : 0;
  }
  return written;
}

void prepare(fixture::Fixture &f, const Mode mode) {
  f.machine.bus().write({.space = DspSpace::x, .address = source, .value = payload});
  f.machine.bus().write({.space = DspSpace::y, .address = destination, .value = canary});
  if (mode != Mode::quiet) {
    f.machine.bus().write({.space = DspSpace::x, .address = control0, .value = software});
    for (unsigned cycle = 0; cycle < 3; ++cycle) {
      static_cast<void>(
          f.machine.advance_dma_reference_cycle(inputs(f.machine, DspDmaGrant::wait)));
    }
    const auto captured =
        f.machine.advance_dma_reference_cycle(inputs(f.machine, DspDmaGrant::available));
    require(captured.phase && captured.phase->kind == DspDmaPhaseKind::source_read &&
                !captured.word,
            "Actual DMA source capture precedes the controller store and its F2 overlap");
    f.machine.bus().write({.space = DspSpace::x, .address = source, .value = canary});
  }
  f.attach();
  for (unsigned index = 0; index < 2; ++index) {
    f.fetch->issue();
    f.fetch->capture();
  }
  f.fetch->issue();
}

bool busy_address(const Case input) {
  return input.mode == Mode::held && input.address <= source0 && input.address >= source0 - 2;
}

void suffix(OwnDspMachine &machine, OwnDspFetch &fetch, const Case input) {
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto grant = input.mode == Mode::held ? DspDmaGrant::wait : DspDmaGrant::available;
  if (input.address == status) {
    const auto cycle = machine.bus().last_dma_cycle();
    const auto message = blofeld::test::esai::rejects([&] {
      static_cast<void>(fetch.advance_dma_automatic(inputs(machine, grant), input.timing));
    });
    require(message.contains("X:0xfffff4") && message.contains("pc=0x000100") &&
                machine.cpu().registers() == before && machine.cpu().progress() == progress &&
                machine.bus().last_dma_cycle() == cycle,
            "Read-only controller status keeps its contextual bus-resource refusal before I/O");
    return;
  }
  const auto result = fetch.advance_dma_automatic(inputs(machine, grant), input.timing);
  if (!result.native) { throw Fault("Controller write lost its actual native effect"); }
  const bool rejected = busy_address(input);
  require(result.captured && result.native && !result.dma.core_stall &&
              machine.clock().elapsed_ticks() == ticks + period,
          "Actual program F2 and controller write share the admitted reference period");
  require(result.dma.word.has_value() == (input.mode == Mode::committed),
          "Only the available captured destination commits before the native register write");
  if (result.dma.word) {
    require(result.dma.word->value == payload,
            "DMA publishes the originally captured payload after source RAM changes");
  }
  if (rejected) {
    const auto &failure = machine.io().failure();
    require(result.native->fault.kind == dsp56300::FaultKind::memory_write && failure &&
                failure->operation == OwnDspIo::Operation::write &&
                failure->address == input.address && failure->pc == fixture::program &&
                machine.cpu().registers() == before && machine.cpu().progress() == progress,
            "Busy address writes retain their actual native I/O fault");
    return;
  }
  require(result.native->fault.kind == dsp56300::FaultKind::none &&
              machine.bus().read({.space = DspSpace::x, .address = input.address}) ==
                  value(input.address),
          "The real DMA register stores its independently specified value");
  bool retired = result.native->retired;
  unsigned periods = 1;
  while (!retired && periods < instruction_bound) {
    const auto next = fetch.advance_dma_automatic(inputs(machine, grant), input.timing);
    fixture::healthy(next);
    retired = next.native && next.native->retired;
    ++periods;
  }
  auto expected = before;
  expected.pc += 2;
  require(retired && machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 1 &&
              machine.cpu().progress().estimated_cycles == progress.estimated_cycles + 2,
          "Native MOVEP immediate changes only PC and retires with its two independent base units");
  if (input.timing == OwnDspAutomaticTiming::native_cycle_budget) {
    require(periods == 2, "Native controller MOVEP retires at its second actual period");
  }
  require(machine.bus().read({.space = DspSpace::y, .address = destination}) ==
              (input.mode == Mode::committed ? payload : canary),
          "Quiet and held destinations remain untouched by the independent controller store");
  if (input.mode == Mode::held) {
    const auto released = fetch.advance_dma_pipeline(
        inputs(machine, DspDmaGrant::available), false, OwnDspNativeCycleModel::unqualified);
    require(released.dma.word && released.dma.word->value == payload &&
                machine.cpu().registers() == expected,
            "A later grant completes the held real word without replaying its controller write");
  }
}

void execute(const Case input) {
  std::array<std::uint32_t, 32> words{};
  words[0] = movep_immediate | (input.address - peripheral_high);
  words[1] = value(input.address);
  fixture::Fixture f(words,
                     {.transfer = {.source = source, .destination = destination, .control = 0}});
  prepare(f, input.mode);
  suffix(f.machine, *f.fetch, input);
}

void run_case(const Case input) {
  try {
    execute(input);
  } catch (const std::exception &error) {
    throw Fault("Controller write X:" + hex(input.address, 6) +
                " mode=" + std::to_string(static_cast<unsigned>(input.mode)) + " timing=" +
                std::to_string(static_cast<unsigned>(input.timing)) + ": " + error.what());
  }
}
}

int main() {
  try {
    unsigned cases = 0;
    for (const auto timing :
         {OwnDspAutomaticTiming::native_cycle_budget,
          OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts}) {
      for (unsigned address = first; address <= last; ++address) {
        for (const auto mode : {Mode::quiet, Mode::held, Mode::committed}) {
          run_case({.address = address, .mode = mode, .timing = timing});
          ++cases;
        }
      }
    }
    for (const auto timing :
         {OwnDspAutomaticTiming::native_cycle_budget,
          OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts}) {
      run_case({.address = status, .mode = Mode::quiet, .timing = timing});
      ++cases;
    }
    std::cout << cases << " authored controller writes preserve native DMA/F2 effects\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
