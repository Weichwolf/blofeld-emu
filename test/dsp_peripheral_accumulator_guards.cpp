// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_accumulator_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <iostream>
#include <string>

namespace blofeld::test::peripheral_accumulator {
namespace {
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned htx = 0xffff93;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned spi_configuration = 0xa9;
constexpr unsigned receive_not_empty = 0x20000;
constexpr unsigned first = 0x800001;
constexpr unsigned second = 0x7fffff;

void spi_word(DspBus &bus, const unsigned value) {
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const bool bit = ((value >> (remaining - 1)) & 1U) != 0;
    bus.shi().observe_spi({.selected = true, .clock = true, .mosi = bit});
    bus.shi().observe_spi({.selected = true, .mosi = bit});
  }
}

void receive(DspBus &bus) {
  bus.shi().observe_spi({.mosi = false});
  bus.write({.space = DspSpace::x, .address = hcsr, .value = spi_configuration});
  bus.write({.space = DspSpace::x, .address = htx, .value = first});
  bus.write({.space = DspSpace::x, .address = htx, .value = second});
  bus.shi().observe_spi({.selected = true, .mosi = false});
  spi_word(bus, first);
  spi_word(bus, second);
}

struct Oracle {
  DspDebugState expected;
  unsigned captures = 0;
};

void fifo_step(DspFetchRun &run, DspMachine &machine, const bool b, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto clock = cycle.completed_cycles;
  if (cycle.bus.peripheral_read) { ++oracle.captures; }
  if (clock == 6) { load_result(oracle.expected, {.value = first, .b = b}); }
  if (clock == 7) { load_result(oracle.expected, {.value = second, .b = !b}); }
  require(phase.retirement_published == (clock >= 7),
          "Two destructive MOVEP loads have genuine consecutive one-period retirements");
  if (clock >= 7) {
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
  }
  require(machine.cpu().debug_state() == oracle.expected &&
              ((machine.bus().read({.space = DspSpace::x, .address = hcsr}) & receive_not_empty) !=
               0) == (clock < 5),
          "FIFO pops at actual AG1; native full fields publish at E1 and flags/PC/counters at E2");
}

void fifo_window(const bool b, const unsigned cut) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, {.status = masked_status | all_ccr})};
  for (const unsigned offset : {0U, 1U}) {
    const std::string reg = (b != (offset != 0)) ? "b" : "a";
    const auto words = checkpoint::assemble(("movep x:<<$ffff94," + reg).c_str());
    machine.bus().write({.address = entry + offset, .value = words.front()});
  }
  receive(machine.bus());
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  while (run.completed_cycles() < cut) { fifo_step(run, machine, b, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    oracle = prefix;
    while (run.completed_cycles() < 8) { fifo_step(run, machine, b, oracle); }
    require(oracle.captures == 2,
            "Each full-accumulator instruction consumes exactly one actual FIFO word");
    static_cast<void>(checkpoint::rejects(
        [&] { static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = hrx})); },
        "Actual full-accumulator reads must leave the FIFO empty"));
  }
}

void unknown(const bool b, const bool partial) {
  DspMachine machine;
  const Input input{
      .port = partial ? 3U : 0U, .value = first, .status = masked_status | all_ccr, .b = b};
  const auto before = prepare(machine, input, false);
  if (partial) {
    auto &gpio = machine.bus().gpio(DspPort::e);
    gpio.write({.reg = DspGpioRegister::control, .value = 1});
    gpio.write({.reg = DspGpioRegister::direction});
    gpio.set_drive({.mask = 1, .levels = 1});
  }
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unknown full peripheral word must not become a successful accumulator value");
    const auto port = ports.at(input.port);
    require(error.contains("read ") && error.contains(hex(port.address, 6)) &&
                error.contains("pc=") && error.contains("omr=") && run.faulted() &&
                run.completed_cycles() == 3 && machine.cpu().debug_state() == before,
            "Whole-word failure retains address/operation/context and all original accumulator/CPU "
            "state");
    run.restore(saved, machine);
    seed_source(machine, input);
    pipeline::clocks(run, machine, 4);
    auto expected = before;
    load_result(expected, input);
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
    require(machine.cpu().debug_state() == expected,
            "Trusted restore plus a real known word enables the native full-accumulator load");
  }
}

void encodings() {
  constexpr unsigned raw_difference = 0x200;
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (const bool b : {false, true}) {
      const auto full = opcode({.port = port, .b = b});
      const auto expected = b ? DspMoveFields{.b = 7} : DspMoveFields{.a = 7};
      const auto raw = b ? DspMoveFields{.b = 2} : DspMoveFields{.a = 2};
      require(decode_move_writeback(full) == expected &&
                  decode_move_writeback(full - raw_difference) == raw,
              "Full A/B encodings remain distinct from raw A1/B1 field-only publication");
    }
  }
}
}

void guard_checks() {
  encodings();
  for (const bool b : {false, true}) {
    unknown(b, false);
    unknown(b, true);
    for (unsigned cut = 0; cut <= 8; ++cut) { fifo_window(b, cut); }
  }
  std::cout << "18 two-word FIFO cuts/36 suffixes;4 strict unknown/partial-word cases and "
               "raw-field controls passed\n";
}
}
