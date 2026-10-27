// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_memory_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::parallel_memory {
namespace {
using native_access::fault;
inline constexpr std::uint64_t pair = 0x234567345678;
inline constexpr unsigned y_peripheral = 0xffffa0;

void second_data_failure() {
  DspMachine machine;
  const std::array<std::string, 3> code{
      "add x1,a", "move x0,x:(r0)+ y:(r4)+,y0", "move x:(r0)+,x0 y:(r4)+,y0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = word_x});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto saved = run.checkpoint(machine);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  auto accepted = before;
  accepted.cpu.pc = body + 1;
  accepted.cpu.a = pipeline::unit;
  accepted.cpu.sr = masked_status | unnormalized;
  accepted.cpu.r[0] = 1;
  accepted.cpu.r[4] = 1;
  ++accepted.native_instructions;
  ++accepted.native_cycles;
  require(error.contains("read Y:0x000007") && error.contains("pc=0x000123") &&
              error.contains("omr=") && run.faulted() && machine.cpu().debug_state() == accepted &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) ==
                  memory_pipeline::first &&
              run.completed_cycles() == 6 && run.elapsed_ticks() == 6 * pipeline::period &&
              machine.bus().last_dma_cycle() == 7,
          "Second native data failure preserves older E2, both younger AG2 results and first real "
          "X store");
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = word_x});
  static_cast<void>(
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); }));
  require(machine.bus().read({.space = DspSpace::x, .address = initial}) == word_x,
          "A quarantined partially entered E1 cannot silently retry its first committed store");
  run.restore(saved, machine);
  machine.bus().write({.space = DspSpace::y, .address = initial, .value = word_y});
  machine.bus().write({.space = DspSpace::x, .address = 0, .value = word_x});
  machine.bus().write({.space = DspSpace::y, .address = 0, .value = word_y});
  clocks(run, machine, 3);
  require(
      machine.cpu().state().a == pipeline::unit && machine.cpu().state().pc == body + 3 &&
          machine.cpu().state().r[0] == 1 && machine.cpu().state().r[4] == 1 &&
          machine.cpu().state().x == pipeline::unit + word_x && machine.cpu().state().y == word_y &&
          machine.bus().read({.space = DspSpace::x, .address = initial}) == memory_pipeline::first,
      "Trusted joint recovery restores the genuine prefix then admits both repaired native "
      "operands");
}

void snapshots_through_interlock() {
  const std::array<std::string, 3> code{
      "move l:(r0)+,a", "move a10,l:(r0)+", "move x:(r0)+,x0 y:(r4)+,y0"};
  for (unsigned clock = 3; clock <= 8; ++clock) {
    DspMachine machine;
    prepare(machine, code);
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      const auto value = space == DspSpace::x ? word_x : word_y;
      machine.bus().write({.space = space, .address = initial, .value = value});
      machine.bus().write({.space = space, .address = 0, .value = memory_pipeline::first});
    }
    machine.bus().write({.space = DspSpace::x, .address = 1, .value = word_x});
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto before = machine.cpu().debug_state();
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == before && run.completed_cycles() == clock,
              "Snapshots preserve both actual AG1 inputs and pre/post interlock stage frontiers");
      clocks(run, machine, 10 - clock);
      require(machine.cpu().state().pc == body + 3 && machine.cpu().state().r[0] == 2 &&
                  machine.cpu().state().r[4] == 0 && machine.cpu().state().a == pair &&
                  machine.cpu().state().x == pipeline::unit + word_x &&
                  machine.cpu().state().y == word_y && run.completed_cycles() == 10 &&
                  run.elapsed_ticks() == 10 * pipeline::period &&
                  machine.bus().read({.space = DspSpace::x, .address = 0}) == word_x &&
                  machine.bus().read({.space = DspSpace::y, .address = 0}) == word_y,
              "Both capabilities, actual long/XY data, transfer bubble and native result survive "
              "two replays");
    }
  }
}

void peripheral_preflight(const bool fifo) {
  DspMachine machine;
  const std::array<std::string, 2> code{
      "add x1,a", fifo ? "move x:(r0)+,x0 y:(r4)+,y0" : "move x0,x:(r0)+ y:(r4)+,y0"};
  const auto primary = fifo ? reference::fifo : initial;
  const auto secondary = fifo ? initial : y_peripheral;
  prepare(machine, code, {.primary = {.pointer = primary}, .secondary_pointer = secondary});
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = word_x});
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x,
                       .address = memory_pipeline::shi_control,
                       .value = memory_pipeline::spi_configuration});
  reference::send(machine.bus().shi(), word_x);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("pc=0x000123") && error.contains("omr=") &&
              error.contains(fifo ? "0xffff94" : "0xffffa0") && run.completed_cycles() == 6 &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 6 &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) == word_x &&
              machine.bus().read({.space = DspSpace::x, .address = reference::fifo}) == word_x,
          "All three resources preflight before DMA/F2/E2/AG2/data and preserve real "
          "FIFO/first-store values");
}

void unsupported_formats() {
  for (const auto *text : {"mpy y0,x0,a l:(r0)+,a",
                           "move ab,l:(r0)+",
                           "move x:(r0)+,a y:(r4)+,b",
                           "move l:(r0+n0),x"}) {
    DspMachine machine;
    const std::array code{std::string(text)};
    prepare(machine, code);
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 2);
    const auto before = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(
        error.contains("automatic DSP decode") && error.contains("pc=") && error.contains("omr=") &&
            machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 2,
        "Unqualified status/overlapping/offset forms refuse at decode before native side effects");
  }
}
}

void state_checks() {
  second_data_failure();
  snapshots_through_interlock();
  peripheral_preflight(false);
  peripheral_preflight(true);
  unsupported_formats();
}
}
