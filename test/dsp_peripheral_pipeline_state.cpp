// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <fstream>
#include <ios>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::peripheral_pipeline {
namespace {
using native_access::fault;
constexpr unsigned changed_literal = 0x654321;
constexpr unsigned register_movep = 0x08c43c;
constexpr unsigned prefix_clocks = 29;
constexpr unsigned prefix_native_cycles = 21;
constexpr unsigned spi_mode_branch = 0xff0016;
constexpr unsigned boot_spi_control = 0xa9;
constexpr unsigned clear_status = 0x14;
constexpr unsigned condition_codes = 0xff;

struct WriteFailure {
  std::string_view instruction;
  std::string_view reason;
  std::string_view access = "write X:";
};

void captured_literal(const bool captured) {
  DspMachine machine;
  const std::array<std::string, 1> code{"movep #>$234567,x:<<$fffffc"};
  prepare(machine, code);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, captured ? 4 : 2);
  machine.bus().write({.address = body + 1, .value = changed_literal});
  clocks(run, machine, captured ? 3 : 5);
  require(
      machine.cpu().state().pc == body + 2 &&
          machine.bus().once_latch() == (captured ? literal : changed_literal),
      "Peripheral E1 consumes its actual F2 literal; only a future F2 sees changed program RAM");
}

void snapshots() {
  const std::array<std::string, 1> code{"movep #>$234567,x:<<$fffffc"};
  for (unsigned clock = 3; clock <= 6; ++clock) {
    DspMachine machine;
    prepare(machine, code);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto at_checkpoint = machine.cpu().debug_state();
    const auto latch = machine.bus().once_latch();
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == at_checkpoint && machine.bus().once_latch() == latch,
              "Joint restore retains actual extension, address kind, phase and peripheral latch");
      clocks(run, machine, 7 - clock);
      auto expected = before;
      expected.cpu.pc = body + 2;
      ++expected.native_instructions;
      expected.native_cycles += 2;
      require(machine.cpu().debug_state() == expected && machine.bus().once_latch() == literal &&
                  run.completed_cycles() == 7,
              "Both native continuations publish exactly the captured peripheral write and result");
    }
  }
}

void rejected_write(const WriteFailure &sample) {
  DspMachine machine;
  const std::array code{std::string(sample.instruction)};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  const auto pll = machine.bus().pll().read();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains(sample.reason) && error.contains(sample.access) &&
                error.contains("pc=") && error.contains("omr=") && run.faulted() &&
                run.completed_cycles() == 5 && machine.cpu().debug_state() == before &&
                !machine.bus().once_latch() && machine.bus().pll().read() == pll,
            "Unknown/invalid actual peripheral write fails contextually without elapsed success");
    if (replay == 0) { first = error; }
    require(error == first,
            "Joint replay preserves the complete native peripheral failure frontier");
  }
}

void register_source_stays_unqualified() {
  DspMachine machine;
  const std::array<std::string, 1> code{"movep x0,x:<<$fffffc"};
  prepare(machine, code);
  // Independent Freescale encoding; register-source operand timing is separate.
  require(machine.bus().peek_program(body) == register_movep,
          "Register MOVEP boundary uses the independent one-word encoding");
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 2);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("automatic DSP decode") && error.contains("pc=") &&
              error.contains("omr=") && run.faulted() && run.completed_cycles() == 2 &&
              machine.cpu().debug_state() == before && !machine.bus().once_latch() &&
              machine.bus().last_dma_cycle() == 2,
          "Register-source MOVEP refuses before any architectural or peripheral effect");
}

void failed_f2_precedes_write() {
  DspMachine machine;
  auto words = checkpoint::assemble("movep #>$234567,x:<<$fffffc");
  words.insert(words.end(), 2, 0);
  fetch_clock::prepare(
      machine, words, {.execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("P:0x000104") && run.faulted() && run.completed_cycles() == 5 &&
              machine.cpu().debug_state() == before && !machine.bus().once_latch(),
          "A real unknown simultaneous F2 fails before native E1 can write the peripheral");
}

void active_dma_refuses_port() {
  DspMachine machine;
  auto words = checkpoint::assemble("movep #>$234567,x:<<$fffffc");
  words.insert(words.end(), 4, 0);
  fetch_clock::prepare(machine,
                       words,
                       {.transfer = {.source = fetch_clock::program_source,
                                     .destination = fetch_clock::program,
                                     .control = fetch_clock::program_copy},
                        .execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  const std::array footprint{DspMemoryAccess{.target = {.address = fetch_clock::program + 2}},
                             DspMemoryAccess{.target = {.space = DspSpace::x, .address = once},
                                             .operation = DspMemoryOperation::write}};
  const auto arbitration = fault([&] {
    static_cast<void>(
        machine.bus().advance_dma(banks::inputs({.index = 1}),
                                  {.accesses = std::span(footprint), .knowledge = banks::known}));
  });
  require(arbitration.contains("active DMA") && arbitration.contains("write at X:0xfffffc") &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 0 &&
              !machine.bus().once_latch(),
          "Explicit peripheral occupancy with actual enabled DMA refuses before sampling or I/O");
  DspFetchRun run(contract(), machine);
  const auto automatic =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(automatic.contains("X:0xfffff4") && automatic.contains("pc=") &&
              automatic.contains("omr=") && run.faulted() && run.completed_cycles() == 0 &&
              machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 0 &&
              !machine.bus().once_latch(),
          "Automatic immediate MOVEP also refuses active DMA before F2 or any native effect");
}

void reference_port(DspMachine &machine, const char *instruction, const bool dma) {
  const auto words = checkpoint::assemble(instruction);
  fetch_clock::prepare(machine,
                       words,
                       {.transfer = {.source = fetch_clock::program_source,
                                     .destination = fetch_clock::program,
                                     .control = dma ? fetch_clock::program_copy : 0}});
  DspReferenceProgress progress{.pending = machine.reference_pending()};
  for (const auto kind : {DspNativeAccessKind::opcode, DspNativeAccessKind::extension}) {
    require(reference::request(progress).kind == kind,
            "The public reference caller consumes the real opcode and immediate extension");
    progress = machine.advance_reference_access(reference::ticket(progress));
  }
  require(reference::request(progress).kind == DspNativeAccessKind::write,
          "The actual native peripheral write remains suspended before arbitration");
}

void reference_idle_write() {
  DspMachine machine;
  reference_port(machine, "movep #>$234567,x:<<$fffffc", false);
  auto expected = machine.cpu().debug_state();
  expected.cpu.pc = fetch_clock::program + 2;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  const auto result =
      machine.advance_reference_dma(banks::inputs({.index = 1}), reference::ticket(machine));
  require(!result.dma.phase && !result.dma.word && !result.dma.core_stall && result.core &&
              result.core->retired && machine.cpu().debug_state() == expected &&
              machine.bus().once_latch() == literal && machine.bus().last_dma_cycle() == 1,
          "The public native-reference caller executes a real idle-DMA peripheral write once");
}

void reference_refusals() {
  for (const bool active : {false, true}) {
    DspMachine machine;
    reference_port(
        machine, active ? "movep #>$234567,x:<<$fffffc" : "movep #>0,x:<<$fffff4", active);
    const auto before = machine.cpu().debug_state();
    const auto ticket = reference::ticket(machine);
    const auto error = fault([&] {
      static_cast<void>(machine.advance_reference_dma(banks::inputs({.index = 1}), ticket));
    });
    require(error.contains(active ? "active DMA" : "write at X:0xfffff4") &&
                error.contains("pc=") && error.contains("omr=") &&
                machine.cpu().debug_state() == before && reference::ticket(machine) == ticket &&
                machine.bus().last_dma_cycle() == 0 && !machine.bus().once_latch(),
            "Native-reference active DMA and controller writes refuse before sampling or I/O");
  }
  DspMachine machine;
  reference_port(machine, "movep #>$234567,y:<<$fffffc", false);
  const auto error = fault([&] {
    static_cast<void>(
        machine.advance_reference_dma(banks::inputs({.index = 1}), reference::ticket(machine)));
  });
  const auto quarantined = fault([&] { machine.cpu().step(); });
  require(error.contains("Unimplemented DSP peripheral write at Y:0xfffffc") &&
              error.contains("pc=") && error.contains("omr=") &&
              quarantined.contains("requires reset") && machine.bus().last_dma_cycle() == 1 &&
              !machine.bus().once_latch(),
          "Resource admission never supplies a register implementation or aliases Y to X");
}
}

void state_checks() {
  reference_idle_write();
  reference_refusals();
  active_dma_refuses_port();
  register_source_stays_unqualified();
  captured_literal(false);
  captured_literal(true);
  snapshots();
  rejected_write({.instruction = "movep #>$234567,x:<<$ffff80",
                  .reason = "Unimplemented DSP peripheral write"});
  rejected_write(
      {.instruction = "movep #>$ffffff,x:<<$fffffd", .reason = "PCTL reserved-bit write"});
  rejected_write({.instruction = "movep #>$234567,y:<<$fffffc",
                  .reason = "Unimplemented DSP peripheral write",
                  .access = "write Y:0xfffffc"});
  failed_f2_precedes_write();
}

void bootstrap(const BootstrapFiles &files) {
  const auto words = read_spi_boot_prefix(files.rom);
  DspMachine machine;
  machine.bus().install_rom({.start = boot, .words = words});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  const auto reset = machine.cpu().debug_state();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  std::ofstream output(files.trace);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  for (unsigned clock = 0; clock < prefix_clocks; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto cpu = machine.cpu().debug_state();
    const auto latch = machine.bus().once_latch();
    output << cycle.completed_cycles << " pc=" << hex(cpu.cpu.pc, 6)
           << " instructions=" << cpu.native_instructions << " native-cycles=" << cpu.native_cycles
           << " once=" << (latch ? hex(*latch, 6) : "unknown") << '\n';
    if (cycle.completed_cycles == 10) {
      auto first = reset;
      first.cpu.pc = boot + 4;
      first.native_instructions += 3;
      first.native_cycles += 4;
      require(cpu == first && latch == 0,
              "The initial genuine OnCE write and two NOP retirements retain their old timing");
    }
  }
  auto expected = reset;
  require((reset.cpu.sr & condition_codes) == 0, "Hardware-reset prefix begins with clear CCR");
  // FM13-44 CLR clears A/E/N/V, sets U/Z and preserves C; reset S/L/C are zero.
  expected.cpu.a = 0;
  expected.cpu.sr |= clear_status;
  expected.cpu.r[1] = boot_spi_control;
  expected.cpu.r[5] = 0;
  expected.cpu.pc = spi_mode_branch;
  expected.native_instructions += 11;
  expected.native_cycles += prefix_native_cycles;
  require(machine.cpu().debug_state() == expected && machine.bus().once_latch() == 0,
          "Native OMR branches reach SPI selection with independently expected CLR/R state");
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("Unqualified automatic DSP decode P:0xff0019") && run.faulted() &&
                run.completed_cycles() == prefix_clocks &&
                machine.cpu().debug_state() == expected && machine.bus().once_latch() == 0,
            "Register-source SHI MOVEP remains the explicit frontier; no boot routine is skipped");
    if (replay == 0) { first = error; }
    require(error == first, "Both reset-prefix continuations stop at the identical actual branch");
  }
  output << "frontier " << first << '\n';
}
}
