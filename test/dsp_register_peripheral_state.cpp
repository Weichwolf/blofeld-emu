// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::peripheral_pipeline {
namespace {
using native_access::fault;

void snapshots() {
  for (unsigned clock = 2; clock <= 7; ++clock) {
    DspMachine machine;
    const std::array<std::string, 2> code{"movep r0,x:<<$fffffc", "move x:(r0)+,x0"};
    prepare(machine, code, {.modifier = memory_pipeline::linear_modifier});
    machine.bus().write({.space = DspSpace::x, .address = 7, .value = literal});
    auto expected = machine.cpu().debug_state();
    expected.cpu.pc = body + 2;
    expected.cpu.r[0] = 8;
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | literal;
    expected.native_instructions += 2;
    expected.native_cycles += 2;
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto at_checkpoint = machine.cpu().debug_state();
    const auto latch = machine.bus().once_latch();
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == at_checkpoint && machine.bus().once_latch() == latch,
              "Restore retains actual register-store/younger-AGU phase and committed I/O");
      clocks(run, machine, 8 - clock);
      require(machine.cpu().debug_state() == expected && machine.bus().once_latch() == 7 &&
                  run.completed_cycles() == 8 &&
                  machine.bus().read({.space = DspSpace::x, .address = 7}) == literal,
              "Each joint continuation preserves old written R0, new visible R0 and real data");
    }
  }
}

void captured_opcode(const bool captured) {
  DspMachine machine;
  const std::array<std::string, 1> code{"movep x0,x:<<$fffffc"};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, captured ? 2 : 1);
  const auto replacement = checkpoint::assemble("movep x1,x:<<$fffffc");
  require(replacement.size() == 1, "Own replacement MOVEP has no extension");
  machine.bus().write({.address = body, .value = replacement.front()});
  clocks(run, machine, captured ? 5 : 6);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected &&
              machine.bus().once_latch() == (captured ? memory_pipeline::first : 1U),
          "Native E1 uses the actual F2 opcode source; only a future F2 observes changed RAM");
}

void transmission_once() {
  DspMachine machine;
  const std::array<std::string, 2> code{"movep #>$a9,x:<<$ffff91", "movep x0,x:<<$ffff93"};
  prepare(machine, code);
  machine.bus().shi().observe_spi({.mosi = false});
  auto expected = machine.cpu().debug_state();
  expected.cpu.pc = body + 3;
  expected.native_instructions += 2;
  expected.native_cycles += 3;
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 8);
  constexpr unsigned transmit_empty = 0x8000;
  require((machine.bus().shi().read(control) & transmit_empty) != 0,
          "The genuine register-source HTX write has moved into the real SHI shifter");
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto e2 = stages(run.advance_cycle(machine, fetch_clock::input()));
    require(e2.retirement_published && machine.cpu().debug_state() == expected &&
                (machine.bus().shi().read(control) & transmit_empty) != 0,
            "Restored E2 retires without replaying a register HTX write into a second word");
  }
}

void refused_forms() {
  for (const auto *text : {"movep sr,x:<<$fffffc",
                           "movep ssh,x:<<$fffffc",
                           "movep a,x:<<$fffffc",
                           "movep b,x:<<$fffffc",
                           "movep a0,x:<<$fffffc",
                           "movep x:<<$ffff94,a"}) {
    DspMachine machine;
    const std::array<std::string, 1> code{text};
    prepare(machine, code);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 2);
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("automatic DSP decode P:") && error.contains("pc=") &&
                error.contains("omr=") && run.faulted() && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before && !machine.bus().once_latch(),
            "Control/stack/accumulator store sources and full-accumulator loads remain explicit "
            "frontiers");
  }
}

void port_load_agu_dependencies() {
  for (const auto *reg : {"r0", "n0", "m0"}) {
    const std::string destination(reg);
    const bool offset = destination == "n0";
    DspMachine machine;
    const std::array<std::string, 2> code{"movep x:<<$ffff90," + destination,
                                          offset ? "move x:(r0)+n0,x0" : "move x:(r0)+,x0"};
    prepare(machine, code);
    auto expected = machine.cpu().debug_state();
    const unsigned address = destination == "r0" ? 1U : 7U;
    machine.bus().write({.space = DspSpace::x, .address = address, .value = literal});
    expected.cpu.r[0] = destination == "r0" ? 2U : 6U;
    if (offset) {
      expected.cpu.r[0] = 0;
      expected.cpu.n[0] = 1;
    }
    if (destination == "m0") { expected.cpu.m[0] = 1; }
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | literal;
    expected.cpu.pc = body + 2;
    expected.native_instructions += 2;
    expected.native_cycles += 2;
    DspFetchRun run(contract(), machine);
    for (unsigned count = 0; count < 32 && machine.cpu().debug_state().native_instructions <
                                               expected.native_instructions;
         ++count) {
      clocks(run, machine, 1);
    }
    require(machine.cpu().debug_state() == expected,
            "Actual native port load must publish the new R/N/M before younger AGU capture: " +
                destination);
  }
}

void strict_write_failure() {
  DspMachine machine;
  const std::array<std::string, 1> code{"movep r0,y:<<$fffffc"};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("write Y:0xfffffc") &&
                error.contains("Unimplemented DSP peripheral write") && error.contains("pc=") &&
                error.contains("omr=") && run.faulted() && run.completed_cycles() == 5 &&
                machine.cpu().debug_state() == before && !machine.bus().once_latch(),
            "Unknown register-source port writes fail in the actual native handler with context");
    if (replay == 0) { first = error; }
    require(error == first, "Trusted replay preserves the exact native write failure frontier");
  }
}

void failed_f2() {
  DspMachine machine;
  auto words = checkpoint::assemble("movep r0,x:<<$fffffc");
  words.insert(words.end(), 3, 0);
  fetch_clock::prepare(
      machine, words, {.execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("P:0x000104") && error.contains("pc=") && error.contains("omr=") &&
              run.faulted() && run.completed_cycles() == 5 &&
              machine.cpu().debug_state() == before && !machine.bus().once_latch(),
          "Unknown simultaneous actual F2 fails before register E1 can commit its write");
}

void active_dma() {
  DspMachine machine;
  auto words = checkpoint::assemble("movep r0,x:<<$fffffc");
  words.insert(words.end(), 4, 0);
  fetch_clock::prepare(machine,
                       words,
                       {.transfer = {.source = fetch_clock::program_source,
                                     .destination = fetch_clock::program,
                                     .control = fetch_clock::program_copy},
                        .execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("DMA arbitration") && error.contains("pc=") && error.contains("omr=") &&
              run.faulted() && run.completed_cycles() == 0 &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 0 &&
              !machine.bus().once_latch(),
          "Register MOVEP never invents a successful active-DMA peripheral sharing contract");
}

void reset_pending() {
  for (const unsigned clock : {4U, 6U}) {
    DspMachine machine;
    const std::array<std::string, 1> code{"movep r0,x:<<$fffffc"};
    prepare(machine, code);
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    machine.cpu().reset_spi_boot();
    const auto reset = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("coordinated") && !machine.reference_fetch().active &&
                !machine.reference_pending() && machine.cpu().debug_state() == reset &&
                run.completed_cycles() == clock,
            "Reset invalidates captured register-store phases and their old clock owner");
    machine.cpu().step();
    auto expected = reset;
    expected.cpu.pc += 2;
    expected.cpu.r[0] = fetch_clock::initial_r0;
    ++expected.native_instructions;
    expected.native_cycles += 2;
    require(machine.cpu().debug_state() == expected,
            "The actual reset ROM entry executes without a stale register-store continuation");
  }
}
}

void register_state_checks() {
  snapshots();
  captured_opcode(false);
  captured_opcode(true);
  transmission_once();
  refused_forms();
  port_load_agu_dependencies();
  strict_write_failure();
  failed_f2();
  active_dma();
  reset_pending();
}
}
