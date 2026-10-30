// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace blofeld::test::peripheral_automatic_guard {
namespace {
constexpr unsigned hckr = 0xffff90;
constexpr unsigned body = memory_pipeline::body;
using memory_pipeline::contract;
using memory_pipeline::prepare;
using pipeline::clocks;

void unknown_source(const std::string &instruction, const std::string_view target) {
  DspMachine machine;
  const std::array code{instruction};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unknown peripheral source reached an invented successful data word");
    require(error.contains(target) && error.contains("read ") && error.contains("pc=") &&
                error.contains("omr=") && run.faulted() && run.completed_cycles() == 3 &&
                machine.cpu().debug_state() == before,
            "Unknown peripheral source lost address/operation/PC/context or published success");
    if (replay == 0) { first = error; }
    require(error == first, "Joint restore moved the unknown peripheral read frontier");
  }
}

void invalid_destinations() {
  for (const auto *reg : {"sr", "omr", "ssh"}) {
    DspMachine machine;
    const std::array<std::string, 1> code{"movep x:<<$ffff94," + std::string(reg)};
    prepare(machine, code);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 2);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unqualified native destination reached destructive FIFO I/O");
    require(error.contains("automatic DSP decode") && error.contains("pc=") &&
                error.contains("omr=") && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before,
            "Control/stack destinations must fail before source capture");
  }
}

void failed_f2_precedes_read() {
  // UM table3-1: default P RAM ends at $0fff. The simultaneous F2 at
  // $1000 has no admitted physical resource, unlike a mapped unknown word.
  constexpr unsigned unmapped = 0x1000;
  constexpr unsigned start = unmapped - 2;
  DspMachine machine;
  auto code = checkpoint::assemble("movep x:<<$ffff90,x0");
  code.push_back(0);
  checkpoint::ram(machine.bus(), start, code);
  DspProgram bootstrap;
  constexpr unsigned long_absolute_jump = 0x0af080; // FM13 absolute JMP extension form.
  bootstrap.word(long_absolute_jump);
  bootstrap.word(start);
  bootstrap.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(machine.bus().advance_dma(banks::inputs({.index = 0}), banks::use()));
  checkpoint::run(machine, 1);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == start, "A genuine native JMP enters the own boundary program");
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unmapped simultaneous F2 reached the early data phase");
    require(error.contains("P:0x001000") && error.contains("Unmapped") && error.contains("pc=") &&
                error.contains("omr=") && run.completed_cycles() == 3 &&
                machine.cpu().debug_state() == before &&
                machine.bus().read({.space = DspSpace::x, .address = hckr}) == 1,
            "A genuinely failed physical F2 precedes the older early read/writeback");
  }
}

void mapped_unknown_f2() {
  DspMachine machine;
  auto code = checkpoint::assemble("movep x:<<$ffff90,x0");
  code.push_back(0);
  fetch_clock::prepare(
      machine, code, {.execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto sample = run.advance_cycle(machine, fetch_clock::input());
    constexpr unsigned absent = fetch_clock::program + 2;
    require(
        sample.fetch.used == 3 && sample.fetch.words.at(2).address == absent &&
            !sample.fetch.words.at(2).data &&
            sample.fetch.words.front().address == fetch_clock::program &&
            sample.fetch.words.front().data && sample.bus.peripheral_read &&
            run.completed_cycles() == 4 && machine.cpu().debug_state() == before,
        "Mapped unknown F2 remains absent while the known older MOVEP captures its real source");
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Absent F2 became an invented known Decode opcode");
    require(error.contains("P:0x000102") && error.contains("pc=") && error.contains("omr=") &&
                run.completed_cycles() == 4 && machine.cpu().debug_state() == before &&
                run.faulted(),
            "The captured absent opcode refuses at actual Decode before any native destination or "
            "retirement");
  }
}

void active_dma() {
  DspMachine machine;
  auto code = checkpoint::assemble("movep x:<<$ffff90,x0");
  code.insert(code.end(), 8, 0);
  fetch_clock::prepare(machine,
                       code,
                       {.transfer = {.source = fetch_clock::program_source,
                                     .destination = fetch_clock::program,
                                     .control = fetch_clock::program_copy},
                        .execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  const std::array footprint{DspMemoryAccess{.target = {.space = DspSpace::x, .address = hckr},
                                             .phase = DspMemoryPhase::peripheral_sample}};
  const auto error = checkpoint::rejects(
      [&] {
        static_cast<void>(machine.bus().advance_dma(
            banks::inputs({.index = 1}),
            {.accesses = std::span(footprint), .knowledge = banks::known}));
      },
      "Active DMA admitted an unqualified early port phase");
  require(error.contains("active DMA") && error.contains("X:0xffff90") &&
              machine.bus().last_dma_cycle() == 0 && machine.cpu().debug_state() == before,
          "Active DMA must reject early peripheral occupancy before request/data effects");
}

void ownership() {
  const std::array<std::string, 1> code{"movep x:<<$ffff90,x0"};
  DspMachine machine;
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  const auto discarded = run.advance_cycle(machine, fetch_clock::input()).bus.peripheral_read;
  if (!discarded) { throw Fault("Actual automatic AG1 lost its owned response ticket"); }
  const auto &discarded_read = *discarded;
  run.restore(saved, machine);
  const auto fresh = run.advance_cycle(machine, fetch_clock::input()).bus.peripheral_read;
  if (!fresh) { throw Fault("Restored automatic AG1 lost its new response ticket"); }
  const auto &fresh_read = *fresh;
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(machine.rebind_reference_peripheral_read(discarded_read)); },
      "Discarded-future automatic capture aliased fresh slot/serial metadata"));
  static_cast<void>(machine.rebind_reference_peripheral_read(fresh_read));
  machine.cpu().reset_spi_boot();
  const auto reset = machine.cpu().debug_state();
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(machine.rebind_reference_peripheral_read(fresh_read)); },
      "Reset retained an automatic peripheral response owner"));
  const auto error = checkpoint::rejects(
      [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
      "Reset allowed a stale automatic pipeline continuation");
  require(error.contains("coordinated") && machine.cpu().debug_state() == reset &&
              !machine.reference_fetch().active && !machine.reference_pending(),
          "Reset must discard native/frontend/peripheral continuation together");
}

void public_phase_bypass() {
  const std::array<std::string, 1> code{"movep x:<<$ffff90,x0"};
  DspMachine machine;
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto before = machine.cpu().debug_state();
  const auto epoch = machine.bus().last_dma_cycle();
  DspReferenceCycleInputs forged{.dma = {.cycle = 4,
                                         .known_requests = 15,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = DspDmaGrant::available},
                                 .peripheral_capture = body,
                                 .order = DspReferenceBusOrder::dma_then_fetch_then_native};
  const auto rejected =
      checkpoint::rejects([&] { static_cast<void>(machine.advance_reference_bus_cycle(forged)); },
                          "Public reference input forged an automatic destructive sample phase");
  require(rejected.contains("actual automatic fetch owner") &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == epoch,
          "Public phase forgery must fail before native/frontend/DMA/peripheral effects");
  clocks(run, machine, 1);
  const auto captured_epoch = machine.bus().last_dma_cycle();
  const auto captured = machine.cpu().debug_state();
  forged.peripheral_capture.reset();
  const auto mixed =
      checkpoint::rejects([&] { static_cast<void>(machine.advance_reference_bus_cycle(forged)); },
                          "Public reference caller adopted an active automatic response owner");
  require(mixed.contains("actual automatic fetch owner") &&
              machine.cpu().debug_state() == captured &&
              machine.bus().last_dma_cycle() == captured_epoch,
          "An active pipeline sample must keep its actual automatic bus-cycle owner");
  clocks(run, machine, 3);
  auto expected = before;
  expected.cpu.pc = body + 1;
  expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | 1U;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Rejected public phase attempts must preserve the valid automatic continuation");
}

void guards() {
  unknown_source("movep x:<<$ffff96,x0", "X:0xffff96");
  unknown_source("movep x:<<$ffff94,a", "X:0xffff94");
  unknown_source("movep x:<<$ffff94,b", "X:0xffff94");
  unknown_source("movep y:<<$fffffc,x0", "Y:0xfffffc");
  unknown_source("jset #0,x:<<$ffff96," + number(body + 2), "X:0xffff96");
  invalid_destinations();
  failed_f2_precedes_read();
  mapped_unknown_f2();
  active_dma();
  ownership();
  public_phase_bypass();
}
}
}

int main(const int argc, const char *const argv[]) {
  try {
    blofeld::test::require(argc == 2, "Select automatic peripheral guards");
    if (std::string_view(argv[1]) == "guards") {
      blofeld::test::peripheral_automatic_guard::guards();
    } else {
      throw blofeld::Fault("Unknown automatic peripheral guard group");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
