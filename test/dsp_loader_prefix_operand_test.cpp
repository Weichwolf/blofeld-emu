// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_spi_prefix_fixture.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned boot = 0xff0000;
constexpr unsigned length_poll = 0xff001a;
constexpr unsigned address_poll = 0xff001d;
constexpr unsigned loop_setup = 0xff0021;
constexpr unsigned data_poll = 0xff0023;
constexpr unsigned program_store = 0xff0025;
constexpr unsigned loop_last = 0xff0026;
constexpr unsigned destination = 0x100;
constexpr unsigned initial_program_word = 0xabcdef;
constexpr unsigned program_words = 2;
constexpr unsigned payload_word = 0;
constexpr unsigned control = 0xffff91;
constexpr unsigned not_empty = 0x20000;
constexpr unsigned clear_status = 0x14;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned empty_cycles = 36;
constexpr unsigned maximum_seek = 40;
constexpr unsigned maximum_store_seek = 24;

void check_state(const DspDebugState &actual, const DspDebugState &expected) {
  require(actual.cpu == expected.cpu && actual.sc == expected.sc && actual.ep == expected.ep &&
              actual.sz == expected.sz && actual.stack_high == expected.stack_high &&
              actual.stack_low == expected.stack_low &&
              actual.native_instructions == expected.native_instructions,
          "Document prefix CPU and retained frames match independent operand/DO/poll expectations");
}

DspFetchCycleResult step(DspFetchRun &run, DspMachine &machine, std::ostream &trace) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto cpu = machine.cpu().state();
  const auto &phase = pipeline::stages(cycle);
  trace << cycle.completed_cycles << " pc=" << hex(cpu.pc, 6) << " lc=" << hex(cpu.lc, 6)
        << " sp=" << hex(cpu.sp, 6) << " r0=" << hex(cpu.r[0], 6) << " r1=" << hex(cpu.r[1], 6);
  if (phase.decode) { trace << " decode=" << hex(phase.decode->address, 6); }
  if (phase.execute1) { trace << " e1=" << hex(phase.execute1->address, 6); }
  if (phase.execute2) { trace << " e2=" << hex(phase.execute2->address, 6); }
  trace << '\n';
  return cycle;
}

DspDebugState
seek_loop(DspFetchRun &run, DspMachine &machine, const DspDebugState &reset, std::ostream &trace) {
  unsigned polls = 0;
  for (unsigned index = 0; index < maximum_seek; ++index) {
    const auto cycle = step(run, machine, trace);
    const auto &phase = pipeline::stages(cycle);
    if (!phase.execute2 || !phase.retirement_published) { continue; }
    if (phase.execute2->address == address_poll) { ++polls; }
    if (phase.execute2->address != loop_setup) { continue; }
    auto expected = reset;
    expected.cpu.pc = data_poll;
    expected.cpu.a = program_words;
    expected.cpu.r[0] = destination;
    expected.cpu.r[1] = destination;
    expected.cpu.r[5] = 0;
    expected.cpu.la = loop_last;
    expected.cpu.lc = program_words;
    expected.cpu.sp += 2;
    expected.cpu.stack_pc = data_poll;
    expected.cpu.stack_sr = reset.cpu.sr | clear_status;
    expected.sc += 2;
    expected.cpu.sr |= clear_status | loop_flag;
    expected.stack_high.at(1) = reset.cpu.la;
    expected.stack_low.at(1) = reset.cpu.lc;
    expected.stack_high.at(2) = data_poll;
    expected.stack_low.at(2) = reset.cpu.sr | clear_status;
    constexpr unsigned before_address_instructions = 19;
    expected.native_instructions += before_address_instructions + polls + 3;
    check_state(machine.cpu().debug_state(), expected);
    require((machine.bus().shi().read(control) & not_empty) == 0,
            "Real HRX reads consume exactly the length and starting-address SPI words");
    return expected;
  }
  throw Fault("Actual document prefix did not retire its native DO A0 setup");
}

void empty_body(DspFetchRun &run,
                DspMachine &machine,
                DspDebugState expected,
                std::ostream &trace) {
  unsigned branches = 0;
  for (unsigned index = 0; index < empty_cycles; ++index) {
    const auto cycle = step(run, machine, trace);
    const auto &phase = pipeline::stages(cycle);
    if (phase.retirement_published) {
      require(phase.execute2 && phase.execute2->address == data_poll,
              "Empty FIFO executes only the genuine body HRNE wait, without program stores");
      ++branches;
      ++expected.native_instructions;
    }
    check_state(machine.cpu().debug_state(), expected);
    require(machine.bus().inspect_program(destination) == initial_program_word &&
                (machine.bus().shi().read(control) & not_empty) == 0,
            "Empty real FIFO preserves the declared RAM sentinel and actual loop count");
  }
  require(branches >= 4,
          "Several real native conditional backedges must execute while HRX is empty");
}

void store_frontier(DspFetchRun &run, DspMachine &machine, std::ostream &trace) {
  spi_prefix::word(machine.bus(), payload_word);
  for (unsigned index = 0; index < maximum_store_seek; ++index) {
    try {
      static_cast<void>(step(run, machine, trace));
    } catch (const Fault &error) {
      const std::string message = error.what();
      require(message.contains("Unqualified automatic DSP decode P:" + hex(program_store, 6)) &&
                  message.contains("pc=" + hex(program_store, 6)) && message.contains("omr=") &&
                  run.faulted() && machine.cpu().state().r[0] == destination &&
                  machine.cpu().state().lc == program_words &&
                  machine.bus().inspect_program(destination) == initial_program_word &&
                  (machine.bus().shi().read(control) & not_empty) != 0,
              "The unsupported P-store remains visible before FIFO consumption or RAM/AGU effects");
      trace << "Expected strict next frontier: " << message << '\n';
      return;
    }
  }
  throw Fault("Expected the genuine unsupported peripheral-to-P store frontier");
}

struct PrefixFiles {
  std::filesystem::path rom;
  std::filesystem::path trace;
};

void run(const PrefixFiles &files) {
  DspMachine machine;
  const auto words = read_spi_boot_prefix(files.rom);
  machine.bus().install_rom({.start = boot, .words = words});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  const auto reset = machine.cpu().debug_state();
  machine.bus().write({.address = destination, .value = initial_program_word});
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  machine.bus().shi().observe_spi({.mosi = false});
  auto inputs = pipeline::contract();
  inputs.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun owner(inputs, machine);
  std::ofstream trace(files.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  for (unsigned index = 0; index < spi_prefix::empty_poll_clocks; ++index) {
    static_cast<void>(step(owner, machine, trace));
  }
  require(machine.cpu().state().pc == length_poll,
          "Unmodified document prefix reaches its first real HRNE poll");
  spi_prefix::word(machine.bus(), program_words);
  pipeline::clocks(owner, machine, 10);
  require(owner.completed_cycles() == spi_prefix::length_received_clocks &&
              machine.cpu().state().pc == address_poll && machine.cpu().state().a == program_words,
          "Real SPI length is in A0 at the independently retained address-poll boundary");
  spi_prefix::word(machine.bus(), destination);
  const auto opened = seek_loop(owner, machine, reset, trace);
  const auto saved = owner.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    owner.restore(saved, machine);
    check_state(machine.cpu().debug_state(), opened);
    require(!owner.faulted() && (machine.bus().shi().read(control) & not_empty) == 0,
            "Joint restore replaces the faulted owner and discards future FIFO/P-store state");
    empty_body(owner, machine, opened, trace);
    store_frontier(owner, machine, trace);
  }
}
}

int main(const int argc, const char *const *const argv) {
  try {
    require(argc == 3, "Expected the pinned local document SPI prefix and private trace path");
    run({.rom = argv[1], .trace = argv[2]});
    std::cout << "Document-derived SPI prefix executes actual address copy/DO A0 and real empty "
                 "body waits\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
