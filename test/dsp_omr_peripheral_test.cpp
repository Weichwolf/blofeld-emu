// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_spi_prefix_fixture.hpp"
#include "native_access_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned sr_move = 0x05f439;
constexpr unsigned seed_x1 = 0x45f400;
constexpr unsigned omr_opcode = 0x04c5ba;
constexpr unsigned body = boot + 6;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned first = 0x123456;
constexpr unsigned second = 0x654321;
constexpr unsigned hrne = 0x20000;
constexpr unsigned tail = 32;
constexpr unsigned last = 11;

DspDebugState prepare(DspMachine &machine, const unsigned value) {
  DspProgram code;
  code.word(sr_move);
  code.word(masked_status);
  code.word(seed_x1);
  code.word(value);
  code.emit("movep #>$a9,x:<<$ffff91");
  code.word(0);
  code.word(omr_opcode);
  code.emit("movep x:<<$ffff94,x0");
  code.emit("movep x:<<$ffff94,y0");
  for (unsigned index = 0; index < tail; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  checkpoint::run(machine, 3);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.omr == reset_omr,
          "Actual native SPI configuration precedes the unchanged reset-context stream");
  spi_prefix::word(machine.bus(), first);
  spi_prefix::word(machine.bus(), second);
  require((machine.bus().shi().read(hcsr) & hrne) != 0,
          "Two real SPI words enter the receive FIFO before clocked AG1 capture");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspFetchRunInputs contract() {
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  return input;
}

void step(DspFetchRun &run, DspMachine &machine, DspDebugState &expected, const unsigned value) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  if (cycle.completed_cycles == 8) {
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | first;
  }
  if (cycle.completed_cycles == 9) {
    expected.cpu.y = (expected.cpu.y & ~std::uint64_t{word_mask}) | second;
  }
  if (phase.retirement_published) {
    if (!phase.execute2) { throw Fault("Native E2 lost the actual instruction word"); }
    require(phase.execute2->address == expected.cpu.pc,
            "Native OMR and both port readers preserve genuine retirement order");
    if (phase.execute2->address == body + 1) { expected.cpu.omr = value; }
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  require(machine.cpu().debug_state() == expected,
          "Whole native state follows old-context physical samples and new-context consumption");
  require(((machine.bus().shi().read(hcsr) & hrne) != 0) == (cycle.completed_cycles < 7),
          "Second actual AG1 empties FIFO before OMR publication without a duplicated pop");
}

struct Case {
  unsigned value;
  unsigned cut;
};

void one_case(const Case input) {
  const auto value = input.value;
  const auto cut = input.cut;
  DspMachine machine;
  auto expected = prepare(machine, value);
  DspFetchRun run(contract(), machine);
  while (run.completed_cycles() < cut) { step(run, machine, expected, value); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    expected = prefix;
    while (run.completed_cycles() < last) { step(run, machine, expected, value); }
    require(expected.cpu.pc == body + 5 && run.elapsed_ticks() == last * pipeline::period,
            "Both real FIFO responses finish at ordinary nominal issue spacing");
    const auto error = native_access::fault(
        [&] { static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = hrx})); });
    require(error.contains("empty"), "Both owned FIFO samples leave no invented third word");
  }
}
}

int main() {
  try {
    for (const auto value : {0x305U, 0x389U, 0x600389U}) {
      for (unsigned cut = 4; cut <= 10; ++cut) { one_case({.value = value, .cut = cut}); }
    }
    std::cout << "42 restored OMR/SPI FIFO suffixes passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
