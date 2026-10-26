// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_spi_prefix_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned source_clock = 0xffff90;
constexpr unsigned fifo_24 = 0x29;
constexpr unsigned wrong_receive = 0xffff93;
constexpr unsigned active_dma_variant = 4;
constexpr unsigned variant_count = 5;
constexpr unsigned payload = 0x123456;
constexpr unsigned sentinel = 0x654321;
constexpr unsigned destination = 0x100;
constexpr unsigned guard_words = 4;
constexpr unsigned maximum_cycles = 40;
constexpr unsigned increment_program = 0x009814;
constexpr unsigned long_r0 = 0x60f400;
constexpr unsigned do_two = 0x060280;

void initialize(DspMachine &machine, const DspProgram &code) {
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().shi().write({.address = source_clock, .value = 0});
  machine.bus().shi().write({.address = hcsr, .value = fifo_24});
  spi_prefix::word(machine.bus(), payload);
  machine.bus().write({.address = destination, .value = sentinel});
  machine.bus().write({.address = 0, .value = sentinel});
}

void automatic_refuses(const bool loop_end) {
  DspProgram code;
  for (unsigned index = 0; index < guard_words; ++index) { code.word(0); }
  const auto store = boot + guard_words + 2;
  code.word(loop_end ? do_two : long_r0);
  code.word(loop_end ? store : store + 1);
  code.word(increment_program);
  for (unsigned index = 0; index < maximum_cycles; ++index) { code.word(0); }
  DspMachine machine;
  initialize(machine, code);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  for (unsigned index = 0; index < maximum_cycles; ++index) {
    const auto before = machine.cpu().debug_state();
    const auto dma_before = machine.bus().last_dma_cycle();
    try {
      static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    } catch (const Fault &error) {
      const std::string message = error.what();
      const auto *const expected =
          loop_end ? "Unqualified program transfer at owned loop end"
                   : "Unqualified automatic P-write to captured/issued instruction";
      require(
          message.contains(expected) && message.contains("P:") && message.contains("pc=") &&
              message.contains("omr=") && run.faulted() && machine.cpu().debug_state() == before &&
              machine.bus().last_dma_cycle() == dma_before &&
              machine.bus().inspect_program(0) == sentinel &&
              machine.bus().inspect_program(store + 1) == 0 &&
              machine.bus().read({.space = DspSpace::x, .address = hrx}) == payload,
          "Unsupported code visibility/loop placement fails before CPU, FIFO, DMA or P effects");
      return;
    }
  }
  throw Fault("Unsupported automatic P transfer unexpectedly succeeded");
}

std::array<DspMemoryAccess, 3> status_overlap() {
  return {{{.target = {.space = DspSpace::x, .address = hcsr},
            .phase = DspMemoryPhase::peripheral_sample},
           {.target = {.space = DspSpace::x, .address = hrx}},
           {.target = {.space = DspSpace::p, .address = destination},
            .operation = DspMemoryOperation::write}}};
}

void footprint_refuses(const unsigned variant) {
  DspProgram code;
  code.word(0);
  DspMachine machine;
  initialize(machine, code);
  auto accesses = status_overlap();
  switch (variant) {
    case 0: accesses[0].target.address = source_clock; break;
    case 1: accesses[1].target.address = wrong_receive; break;
    case 2: accesses[1].target.space = DspSpace::y; break;
    case 3: accesses[2].phase = DspMemoryPhase::peripheral_sample; break;
    case 4: banks::configure(machine.bus()); break;
    default: throw Fault("Unknown own refusal vector");
  }
  const auto before = machine.bus().last_dma_cycle();
  try {
    static_cast<void>(machine.bus().advance_dma(banks::inputs({.index = 0}), banks::use(accesses)));
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(
        message.contains("Unqualified") &&
            (variant != active_dma_variant || message.contains("active DMA")) &&
            machine.bus().last_dma_cycle() == before &&
            machine.bus().inspect_program(destination) == sentinel &&
            machine.bus().read({.space = DspSpace::x, .address = hrx}) == payload,
        "Unqualified status overlap or active DMA fails before sampling or either device effect");
    return;
  }
  throw Fault("Unqualified peripheral/P/status footprint unexpectedly succeeded");
}
}

int main() {
  try {
    automatic_refuses(false);
    automatic_refuses(true);
    for (unsigned variant = 0; variant < variant_count; ++variant) { footprint_refuses(variant); }
    std::cout << "Seven automatic code/loop and peripheral/status/DMA refusal vectors passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
