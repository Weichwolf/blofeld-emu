// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_transfer_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::transfer_pipeline {
namespace {
using native_access::fault;

void captured_extension(const bool captured) {
  DspMachine machine;
  const std::array<std::string, 2> code{"move #>7,r0", "move x:(r0),x0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = other});
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = replacement});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, captured ? 4 : 2);
  machine.bus().write({.address = body + 1, .value = 1});
  clocks(run, machine, captured ? 7 : 9);
  require(
      machine.cpu().state().pc == body + 3 && machine.cpu().state().r[0] == (captured ? 7U : 1U) &&
          machine.cpu().state().x == pipeline::unit + (captured ? other : replacement) &&
          run.completed_cycles() == 11,
      "Native E1 uses the actual captured F2 extension; only a future F2 sees code replacement");
}

void snapshots_through_agu_hold() {
  const std::array<std::string, 2> code{"move #>15,m0", "move x:(r0)+n0,x0"};
  for (unsigned clock = 3; clock <= 9; ++clock) {
    DspMachine machine;
    prepare(machine, code);
    machine.bus().write({.space = DspSpace::x, .address = 7, .value = other});
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto at_checkpoint = machine.cpu().debug_state();
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == at_checkpoint && run.completed_cycles() == clock,
              "Joint restore retains actual extension/AGU fields and decode hold frontier");
      clocks(run, machine, 11 - clock);
      auto expected = before;
      expected.cpu.r[0] = 10;
      expected.cpu.m[0] = 15;
      expected.cpu.x = pipeline::unit + other;
      expected.cpu.pc = body + 3;
      expected.native_instructions += 2;
      expected.native_cycles += 3;
      require(
          machine.cpu().debug_state() == expected && run.completed_cycles() == 11 &&
              run.elapsed_ticks() == 11 * pipeline::period,
          "Both replays retain actual M caches, later native AGU, data and retirement identity");
    }
  }
}

DspAddressStageTicket capture_actual_address(DspMachine &machine, const unsigned address) {
  DspInstructionPipeline stage;
  stage.profile = DspFetchPipeline::automatic_memory;
  const auto fetch = machine.reference_fetch();
  for (unsigned index = 0; index < fetch.used; ++index) {
    const auto word = fetch.words.at(index);
    if (word.address == address) {
      stage.view.cycle.ag1 = word.require_known({.pc = address, .omr = fetch.omr});
    }
  }
  require(stage.view.cycle.ag1.has_value(), "Explicit AG1 requires the actual earlier F2 word");
  stage.capture_address(machine);
  const auto ticket = stage.addresses[0];
  if (!ticket) { throw Fault("The native AGU lost its actual address capability"); }
  return *ticket;
}

void later_address_publication_survives() {
  DspMachine machine;
  const std::array<std::string, 3> code{"move x:(r0)+,r0", "move x:(r0)+,r0", "move x:(r0)+,x0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = 1});
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = 4});
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  expected.cpu.r[0] = 1;
  require(machine.cpu().debug_state() == expected, "The first actual R load still holds its E2");
  require(machine.reference_fetch().pending_address == body + 2,
          "Decode holds preserve the actual pending F2 of the later consumer");

  // Explicit trusted native boundaries; do not resume the automatic elapsed-clock owner.
  const auto prior = machine.reference_pending();
  if (!prior) { throw Fault("The actual predecessor lost its retirement capability"); }
  static_cast<void>(machine.overlap_reference_instruction(prior.value().ticket));
  const auto load = capture_actual_address(machine, body + 1);
  static_cast<void>(machine.advance_reference_bus_cycle(
      {.dma = banks::inputs({.index = 7}),
       .fetch = DspProgramFetchStage::read,
       .address_update = load,
       .order = DspReferenceBusOrder::dma_then_fetch_then_native}));
  const auto result = machine.advance_reference_bus_cycle(
      {.dma = banks::inputs({.index = 8}),
       .memory = load,
       .order = DspReferenceBusOrder::dma_then_fetch_then_native});
  expected.cpu.r[0] = 4;
  require(result.memory && machine.cpu().debug_state() == expected,
          "Actual second E1 loaded R supersedes its AG2 while the first load remains held");
  const auto later = capture_actual_address(machine, body + 2);
  const auto address = machine.advance_reference_bus_cycle(
      {.dma = banks::inputs({.index = 9}),
       .address_update = later,
       .order = DspReferenceBusOrder::dma_then_fetch_then_native});
  require(address.address_update && address.address_update->data_accesses[0].target.address == 4 &&
              address.address_update->update.value == 5,
          "A genuinely later native AGU uses loaded R and publishes its actual postincrement");
  const auto previous = machine.reference_retirement();
  if (!previous) { throw Fault("The first load lost its separate actual result"); }
  static_cast<void>(machine.retire_reference_overlap(previous.value().ticket));
  expected.cpu.r[0] = 5;
  expected.cpu.pc = body + 1;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Prior E2 reapplication of younger loaded R preserves the genuinely later AG2 value");
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("The actual load lost its own E2 retirement"); }
  const auto retired = machine.advance_reference_access(pending.value().ticket);
  expected.cpu.pc = body + 2;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(retired.retired && machine.cpu().debug_state() == expected,
          "Loaded R retirement preserves the later accepted address without replaying I/O");
}

void addressed_successor_keeps_held_arithmetic() {
  DspMachine machine;
  const std::array<std::string, 2> code{"mpy y0,x0,a", "add x1,a x:(r0)+,x0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = other});
  auto expected = machine.cpu().debug_state();
  auto inputs = contract();
  inputs.pipeline = DspFetchPipeline::explicit_effects;
  DspFetchRun run(inputs, machine);
  clocks(run, machine, 6);

  // Explicit trusted boundaries after real F2; do not resume the elapsed-clock owner.
  const auto opcode = machine.reference_pending();
  if (!opcode) { throw Fault("The actual first opcode lost its admission capability"); }
  static_cast<void>(machine.advance_reference_access(opcode->ticket));
  require(machine.cpu().debug_state() == expected,
          "The real multiplier result remains held before explicit predecessor retirement");
  const auto held = machine.reference_pending();
  if (!held) { throw Fault("The actual multiplier lost its held retirement"); }
  static_cast<void>(machine.overlap_reference_instruction(held->ticket));
  const auto address = capture_actual_address(machine, body + 1);
  static_cast<void>(machine.advance_reference_bus_cycle(
      {.dma = banks::inputs({.index = 7}),
       .address_update = address,
       .order = DspReferenceBusOrder::dma_then_fetch_then_native}));
  const auto loaded = machine.advance_reference_bus_cycle(
      {.dma = banks::inputs({.index = 8}),
       .memory = address,
       .order = DspReferenceBusOrder::dma_then_fetch_then_native});
  expected.cpu.r[0] = 0;
  expected.cpu.x = pipeline::unit + other;
  require(loaded.memory && machine.cpu().debug_state() == expected,
          "Actual addressed E1 publishes only its load and R while both arithmetic results wait");
  const auto previous = machine.reference_retirement();
  if (!previous) { throw Fault("The real multiplier lost its separate predecessor role"); }
  static_cast<void>(machine.retire_reference_overlap(previous->ticket));
  constexpr std::uint64_t product = 0x091a2b000000;
  expected.cpu.a = product;
  expected.cpu.sr = masked_status | unnormalized;
  expected.cpu.pc = body + 1;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "Prior E2 publishes the original product while preserving the actual younger load");
  const auto current = machine.reference_pending();
  if (!current) { throw Fault("The addressed successor lost its actual held result"); }
  require(machine.advance_reference_access(current->ticket).retired,
          "The addressed successor retires its own genuine native candidate");
  constexpr std::uint64_t sum = 0x091a2c000000;
  expected.cpu.a = sum;
  expected.cpu.pc = body + 2;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "The addressed ADD computes from the real held product, never stale visible A or PC");
}

void two_producers_and_dual_consumer() {
  DspMachine machine;
  const std::array<std::string, 3> code{
      "move #<15,m0", "move #<1,n4", "move x:(r0)+n0,x0 y:(r4)+n4,y0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = other});
  machine.bus().write({.space = DspSpace::y, .address = 7, .value = replacement});
  DspFetchRun run(contract(), machine);
  std::vector<unsigned> conflicts;
  for (unsigned clock = 0; clock < 12; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    if (stages(cycle).agu_dependencies != 0) {
      conflicts.push_back(stages(cycle).agu_dependencies);
    }
  }
  const std::vector<unsigned> expected{0x11, 0x11, 0x10};
  require(conflicts == expected && machine.cpu().state().pc == body + 3 &&
              machine.cpu().state().r[0] == 10 && machine.cpu().state().r[4] == 0 &&
              machine.cpu().state().m[0] == 15 && machine.cpu().state().n[4] == 1 &&
              machine.cpu().state().x == pipeline::unit + other &&
              machine.cpu().state().y == replacement,
          "Two explicit producers age independently and both native dual AGUs see ready values");
}

void multiword_gap_ages_actual_clocks() {
  DspMachine machine;
  const std::array<std::string, 3> code{"move x1,r0", "move #>$654321,y1", "move x:(r0)+,x0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = other});
  DspFetchRun run(contract(), machine);
  unsigned conflicts = 0;
  for (unsigned clock = 0; clock < 11; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    if (stages(cycle).agu_dependencies != 0) { ++conflicts; }
  }
  require(conflicts == 1 && machine.cpu().state().pc == body + 4 &&
              machine.cpu().state().r[0] == 2 &&
              machine.cpu().state().x == pipeline::unit + other &&
              machine.cpu().state().y ==
                  (std::uint64_t{replacement} << word_bits) + memory_pipeline::half,
          "One intervening two-word transfer ages two actual clocks, leaving only one decode hold");
}

void unknown_extension_recovery() {
  DspMachine machine;
  const auto assembled = checkpoint::assemble("move #>$234567,r0");
  require(assembled.size() == 2, "Own explicit long immediate has two words");
  const std::array code{assembled.front()};
  fetch_clock::prepare(
      machine, code, {.execution = DspReferenceExecution::staged_register_boundaries});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 2);
  const auto saved = run.checkpoint(machine);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("0x000101") && error.contains("pc=") && error.contains("omr=") &&
              run.faulted() && run.completed_cycles() == 2 && machine.cpu().debug_state() == before,
          "Unknown real F2 extension fails with context before any native destination or elapsed "
          "success");
  run.restore(saved, machine);
  std::vector<std::uint32_t> repaired{other};
  repaired.insert(repaired.end(), 16, 0);
  checkpoint::ram(machine.bus(), fetch_clock::program + 1, repaired);
  clocks(run, machine, 5);
  auto expected = before;
  expected.cpu.r[0] = other;
  expected.cpu.pc = fetch_clock::program + 2;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(
      machine.cpu().debug_state() == expected && run.completed_cycles() == 7,
      "Trusted joint repair fetches and consumes the actual extension once, then retires natively");
}

void unqualified_sources_and_control() {
  for (const auto *text : {"move a0,x0",
                           "move sr,x0",
                           "move n0,x0",
                           "move #>7,sr",
                           "move x:(r0),omr",
                           "move r0,x:(r0)",
                           "move x:>$7,x0"}) {
    DspMachine machine;
    const std::array code{std::string(text)};
    prepare(machine, code);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 2);
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("automatic DSP decode") && error.contains("pc=") &&
                error.contains("omr=") && machine.cpu().debug_state() == before &&
                run.completed_cycles() == 2,
            std::string(text) +
                ": unqualified source/control/absolute forms refuse before side effects");
  }
}
}

void state_checks() {
  captured_extension(false);
  captured_extension(true);
  snapshots_through_agu_hold();
  later_address_publication_survives();
  addressed_successor_keeps_held_arithmetic();
  two_producers_and_dual_consumer();
  multiword_gap_ages_actual_clocks();
  unknown_extension_recovery();
  unqualified_sources_and_control();
}
}

int main(const int argc, char **argv) {
  try {
    using namespace blofeld::test::transfer_pipeline;
    if (argc != 2) { throw blofeld::Fault("Expected fields, agu or state group"); }
    const auto group = std::string_view(argv[1]);
    if (group == "fields") {
      field_checks();
    } else if (group == "agu") {
      dependency_checks();
    } else if (group == "state") {
      state_checks();
    } else {
      throw blofeld::Fault("Unknown automatic transfer group");
    }
    std::cout << "Native automatic transfer " << group << " checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
